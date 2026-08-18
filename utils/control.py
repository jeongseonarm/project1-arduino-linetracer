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
        min_speed=-5.0
    ):

        # =========================
        # 기본 속도
        # =========================

        self.base_speed = base_speed


        # =========================
        # PID Gain
        # =========================

        self.kp = kp
        self.ki = ki
        self.kd = kd


        # =========================
        # Sampling time
        # =========================

        self.dt = dt


        # =========================
        # 센서 간격
        #
        # L = -sensor_spacing
        # C = 0
        # R = +sensor_spacing
        # =========================

        self.sensor_spacing = sensor_spacing


        # =========================
        # 라인 검출 Threshold
        #
        # d_max가 threshold 이상이면
        # 검은 라인을 감지했다고 판단
        # =========================

        self.threshold = threshold


        # =========================
        # 속도 제한
        # =========================

        self.max_speed = max_speed
        self.min_speed = min_speed


        # =========================
        # PID 상태 변수
        # =========================

        self.previous_error = 0.0

        self.integral = 0.0


        # =========================
        # 라인 감지 여부
        # =========================

        self.has_seen_line = False


    def control(
        self,
        sensor_values
    ):

        # =========================
        # 센서 반사율
        # =========================

        r_L, r_C, r_R = sensor_values


        # =========================
        # 검정 정도
        #
        # 흰색 : 1023 -> D = 0
        # 검정 : 0    -> D = 1023
        # =========================

        d_L = (
            1023.0
            - float(r_L)
        )

        d_C = (
            1023.0
            - float(r_C)
        )

        d_R = (
            1023.0
            - float(r_R)
        )


        # =========================
        # 최대 검정 정도
        # =========================

        d_max = max(
            d_L,
            d_C,
            d_R
        )


        # =========================
        # 전체 라인 검출량
        # =========================

        total = (
            d_L
            + d_C
            + d_R
        )


        # =========================
        # Line Error
        #
        # Weighted Average
        #
        # Sensor position
        #
        # L : -s
        # C :  0
        # R : +s
        #
        # error =
        #
        # (-s * d_L
        #  +0 * d_C
        #  +s * d_R)
        #
        # -----------------
        #
        # (d_L + d_C + d_R)
        # =========================

        if d_max >= self.threshold:

            # -------------------------
            # 라인 감지
            # -------------------------

            self.has_seen_line = True


            error = (

                -self.sensor_spacing
                * d_L

                +

                self.sensor_spacing
                * d_R

            ) / total


        else:

            # -------------------------
            # 라인 미감지
            # -------------------------

            if self.has_seen_line:

                # 이전에 라인을 본 경우
                #
                # 마지막 오차 방향으로 탐색

                if self.previous_error > 0:

                    error = (
                        self.sensor_spacing
                    )

                elif self.previous_error < 0:

                    error = (
                        -self.sensor_spacing
                    )

                else:

                    error = 0.0


            else:

                # -------------------------
                # 시작부터 라인을 못 찾은 경우
                #
                # 오른쪽 방향 탐색
                # -------------------------

                error = (
                    self.sensor_spacing
                )


            # -------------------------
            # 라인 유실 시
            # Integral 초기화
            # -------------------------

            self.integral = 0.0


        # =========================
        # Integral
        #
        # 라인이 감지된 경우만 적분
        # =========================

        if d_max >= self.threshold:

            self.integral += (
                error
                * self.dt
            )


            # Integral Windup 방지

            self.integral = np.clip(
                self.integral,
                -10.0,
                10.0
            )


        # =========================
        # Derivative
        # =========================

        derivative = (

            error
            - self.previous_error

        ) / self.dt


        # =========================
        # PID Control
        # =========================

        correction = (

            self.kp
            * error

            +

            self.ki
            * self.integral

            +

            self.kd
            * derivative

        )


        # =========================
        # 이전 Error 저장
        # =========================

        self.previous_error = error


        # =========================
        # 좌우 바퀴 속도
        #
        # error > 0
        #
        # → 오른쪽에 라인
        # → 오른쪽으로 회전
        #
        # v_L 증가
        # v_R 감소
        #
        # =========================

        v_L = (

            self.base_speed
            + correction

        )


        v_R = (

            self.base_speed
            - correction

        )


        # =========================
        # 속도 제한
        # =========================

        v_L = np.clip(
            v_L,
            self.min_speed,
            self.max_speed
        )


        v_R = np.clip(
            v_R,
            self.min_speed,
            self.max_speed
        )


        # =========================
        # Control output
        # =========================

        return np.array(
            [
                v_L,
                v_R
            ]
        )