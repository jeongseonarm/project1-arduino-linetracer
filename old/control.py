import numpy as np


class LineController:

    def __init__(
        self,
        base_speed=5.0,
        kp=2.0,
        ki=0.01,
        kd=0.5,
        dt=0.02,
        threshold=200.0,  # d_max 기준 검은선 인식 최소 임계값
        max_speed=10.0,
        min_speed=-5.0,  # 급회전 및 이탈 복귀를 위한 역방향 속도 허용
    ):

        # =========================
        # 기본 설정
        # =========================

        self.base_speed = base_speed

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.dt = dt

        # 라인 검출 최소량 (d_max 기준)
        self.threshold = threshold

        self.max_speed = max_speed
        self.min_speed = min_speed

        # =========================
        # PID 및 상태 변수
        # =========================

        self.previous_error = 0.0
        self.integral = 0.0
        self.has_seen_line = False  # 주행 중 라인을 감지한 적이 있는지 플래그

    def control(self, sensor_values):

        # =========================
        # 반사율 및 검정 정도 계산
        # =========================

        r_L, r_C, r_R = sensor_values

        d_L = 1023.0 - float(r_L)
        d_C = 1023.0 - float(r_C)
        d_R = 1023.0 - float(r_R)

        # 3개 센서 중 가장 강하게 검은색을 읽은값
        d_max = max(d_L, d_C, d_R)

        # =========================
        # Line error (위치 오차) 계산
        # =========================

        if d_max >= self.threshold:
            # [Case 1] 라인이 정상 감지된 경우
            self.has_seen_line = True

            # 구간별 연속 오차 산출 공식 (-1.0 ~ +1.0)
            if d_C >= d_L and d_C >= d_R:
                # (1) 라인이 중앙 부근에 위치: 범위 (-0.5 ~ +0.5)
                error = (d_R - d_L) / (2.0 * d_C)
            elif d_L > d_R:
                # (2) 라인이 왼쪽에 치우침: 범위 (-1.0 ~ -0.5)
                error = -0.5 - ((d_L - d_C) / (2.0 * d_L))
            else:
                # (3) 라인이 오른쪽에 치우침: 범위 (+0.5 ~ +1.0)
                error = 0.5 + ((d_R - d_C) / (2.0 * d_R))

        else:
            # [Case 2] 라인 이탈 및 미감지 상태 (전부 흰색)
            if self.has_seen_line:
                # A. 주행 중 라인을 놓친 경우: 직전 방향으로 복귀
                error = 1.0 if self.previous_error > 0 else -1.0
            else:
                # B. 처음 시작부터 3개 다 흰색인 경우 (Initial Offset): 오른쪽 탐색 회전
                error = 1.0

            # 라인을 놓친 동안 적분값이 누적되어 튀는 현상(Windup) 방지
            self.integral = 0.0

        # =========================
        # Integral & Derivative
        # =========================

        # 라인이 보일 때만 적분값 누적 및 클리핑
        if d_max >= self.threshold:
            self.integral += error * self.dt
            self.integral = np.clip(self.integral, -2.0, 2.0)

        derivative = (error - self.previous_error) / self.dt

        # =========================
        # PID Correction
        # =========================

        correction = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )

        # 이전 error 저장
        self.previous_error = error

        # =========================
        # 좌우 바퀴 속도 계산
        # =========================

        v_L = self.base_speed + correction
        v_R = self.base_speed - correction

        # 속도 범위 제어 (min_speed를 음수로 두면 급격한 곡선/복귀 시 제자리 회전 가능)
        v_L = np.clip(v_L, self.min_speed, self.max_speed)
        v_R = np.clip(v_R, self.min_speed, self.max_speed)

        return np.array([v_L, v_R])

    