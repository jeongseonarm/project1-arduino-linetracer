import numpy as np


class LineController:

    def __init__(
        self,
        base_speed=5.0,
        kp=2.0,
        ki=0.01,
        kd=0.5,
        dt=0.02,
        sensor_spacing=2.0,
        threshold=200.0,
        max_speed=10.0,
        min_speed=-5.0,
    ):
        self.base_speed = base_speed
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.sensor_spacing = sensor_spacing
        self.threshold = threshold
        self.max_speed = max_speed
        self.min_speed = min_speed

        self.previous_error = 0.0
        self.integral = 0.0
        self.has_seen_line = False

    def control(self, sensor_values):
        r_L, r_C, r_R = sensor_values

        # 밝기 값을 검정 라인 흡수량으로 변환
        d_L = 1023.0 - r_L
        d_C = 1023.0 - r_C
        d_R = 1023.0 - r_R

        d_max = max(d_L, d_C, d_R)
        total = d_L + d_C + d_R

        # 센서 감지 여부 판별 및 라인 오차 계산
        if d_max >= self.threshold:
            self.has_seen_line = True
            if total > 0.0:
                # 좌우 센서 가중 평균을 이용한 오차 산출
                error = (
                    -self.sensor_spacing * d_L + self.sensor_spacing * d_R
                ) / total
            else:
                error = 0.0
        else:
            # 라인 이탈 시 직전 감지 방향으로 복귀 탐색
            if self.has_seen_line:
                if self.previous_error > 0:
                    error = self.sensor_spacing
                elif self.previous_error < 0:
                    error = -self.sensor_spacing
                else:
                    error = 0.0
            else:
                error = self.sensor_spacing

            # 라인 이탈 시 적분값 초기화
            self.integral = 0.0

        # 적분항 누적 및 Anti-windup
        if d_max >= self.threshold:
            self.integral += error * self.dt
            self.integral = np.clip(self.integral, -10.0, 10.0)

        # 미분항 및 PID 제어 보정량 계산
        derivative = (error - self.previous_error) / self.dt
        correction = (
            (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        )
        self.previous_error = error

        # 좌우 바퀴 제어 속도 산출 및 속도 범위 제한
        v_L = np.clip(
            self.base_speed + correction, self.min_speed, self.max_speed
        )
        v_R = np.clip(
            self.base_speed - correction, self.min_speed, self.max_speed
        )

        return np.array([v_L, v_R])