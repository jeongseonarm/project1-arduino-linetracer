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

        # 라인 흡수량 변환
        d_L = 1023.0 - r_L
        d_C = 1023.0 - r_C
        d_R = 1023.0 - r_R

        d_max = max(d_L, d_C, d_R)
        total = d_L + d_C + d_R

        # 오차 산출 및 라인 이탈 처리
        if d_max >= self.threshold:
            self.has_seen_line = True
            if total > 0.0:
                error = (
                    -self.sensor_spacing * d_L + self.sensor_spacing * d_R
                ) / total
            else:
                error = 0.0
        else:
            if self.has_seen_line:
                if self.previous_error > 0:
                    error = self.sensor_spacing
                elif self.previous_error < 0:
                    error = -self.sensor_spacing
                else:
                    error = 0.0
            else:
                error = self.sensor_spacing

            self.integral = 0.0

        # 적분 누적 및 Anti-windup
        if d_max >= self.threshold:
            self.integral += error * self.dt
            self.integral = np.clip(self.integral, -10.0, 10.0)

        # PID 보정량 계산
        derivative = (error - self.previous_error) / self.dt
        correction = (
            (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        )
        self.previous_error = error

        # 오차 부호별 바퀴 속도 제어
        if error < 0:
            v_L = self.base_speed + correction
            v_R = self.base_speed
        else:
            v_L = self.base_speed
            v_R = self.base_speed - correction

        # 속도 범위 제한
        v_L = np.clip(v_L, self.min_speed, self.max_speed)
        v_R = np.clip(v_R, self.min_speed, self.max_speed)

        return np.array([v_L, v_R])