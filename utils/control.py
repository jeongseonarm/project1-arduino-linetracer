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


        # =========================
        # Median Filter
        #
        # 각 센서의 최근 3개 측정값
        # =========================

        self.L_buffer = []
        self.C_buffer = []
        self.R_buffer = []


    # =========================================================
    # 3-point Median Filter
    # =========================================================

    @staticmethod
    def median3(a, b, c):

        return sorted([a, b, c])[1]


    # =========================================================
    # 센서별 Median Filter
    # =========================================================

    def filter_sensor_values(self, sensor_values):

        r_L, r_C, r_R = sensor_values


        # -----------------------------------------------------
        # Buffer에 새로운 센서값 추가
        # -----------------------------------------------------

        self.L_buffer.append(float(r_L))
        self.C_buffer.append(float(r_C))
        self.R_buffer.append(float(r_R))


        # -----------------------------------------------------
        # 초기에는 측정값이 3개가 안 될 수 있으므로
        # 현재 값으로 buffer를 채움
        # -----------------------------------------------------

        if len(self.L_buffer) < 3:

            self.L_buffer = [
                self.L_buffer[-1]
            ] * 3

            self.C_buffer = [
                self.C_buffer[-1]
            ] * 3

            self.R_buffer = [
                self.R_buffer[-1]
            ] * 3


        # -----------------------------------------------------
        # 최근 3개 값만 유지
        # -----------------------------------------------------

        self.L_buffer = self.L_buffer[-3:]
        self.C_buffer = self.C_buffer[-3:]
        self.R_buffer = self.R_buffer[-3:]


        # -----------------------------------------------------
        # Median
        # -----------------------------------------------------

        filtered_L = self.median3(
            self.L_buffer[0],
            self.L_buffer[1],
            self.L_buffer[2]
        )

        filtered_C = self.median3(
            self.C_buffer[0],
            self.C_buffer[1],
            self.C_buffer[2]
        )

        filtered_R = self.median3(
            self.R_buffer[0],
            self.R_buffer[1],
            self.R_buffer[2]
        )


        return np.array([
            filtered_L,
            filtered_C,
            filtered_R
        ])


    # =========================================================
    # Control
    # =========================================================

    def control(self, sensor_values):


        # =====================================================
        # Raw sensor → Median Filter
        # =====================================================

        filtered_values = self.filter_sensor_values(
            sensor_values
        )


        r_L, r_C, r_R = filtered_values


        # =====================================================
        # 검정 정도
        #
        # 흰색 : 1023 → D = 0
        # 검정 : 0    → D = 1023
        # =====================================================

        d_L = 1023.0 - r_L
        d_C = 1023.0 - r_C
        d_R = 1023.0 - r_R


        # =====================================================
        # 최대 검정 정도
        # =====================================================

        d_max = max(
            d_L,
            d_C,
            d_R
        )


        # =====================================================
        # 전체 라인 검출량
        # =====================================================

        total = (
            d_L
            + d_C
            + d_R
        )


        # =====================================================
        # Line Error
        # =====================================================

        if d_max >= self.threshold:

            # -------------------------------------------------
            # 라인 감지
            # -------------------------------------------------

            self.has_seen_line = True


            # -------------------------------------------------
            # Weighted Average
            #
            # L = -S
            # C =  0
            # R = +S
            #
            # error =
            #
            # (-S*d_L + S*d_R)
            # -----------------
            #       total
            # -------------------------------------------------

            if total > 0.0:

                error = (

                    -self.sensor_spacing * d_L

                    +

                    self.sensor_spacing * d_R

                ) / total

            else:

                error = 0.0


        else:

            # -------------------------------------------------
            # 라인 미감지
            # -------------------------------------------------

            if self.has_seen_line:

                # ---------------------------------------------
                # 이전 오차 방향으로 탐색
                # ---------------------------------------------

                if self.previous_error > 0:

                    error = self.sensor_spacing

                elif self.previous_error < 0:

                    error = -self.sensor_spacing

                else:

                    error = 0.0


            else:

                # ---------------------------------------------
                # 시작부터 라인을 못 찾은 경우
                # ---------------------------------------------

                error = self.sensor_spacing


            # -------------------------------------------------
            # Integral Reset
            # -------------------------------------------------

            self.integral = 0.0


        # =====================================================
        # Integral
        # =====================================================

        if d_max >= self.threshold:

            self.integral += (
                error
                * self.dt
            )


            # -------------------------------------------------
            # Integral Windup 방지
            # -------------------------------------------------

            self.integral = np.clip(
                self.integral,
                -10.0,
                10.0
            )


        # =====================================================
        # Derivative
        # =====================================================

        derivative = (

            error
            - self.previous_error

        ) / self.dt


        # =====================================================
        # PID Control
        # =====================================================

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


        # =====================================================
        # 이전 Error 저장
        # =====================================================

        self.previous_error = error


        # =====================================================
        # 좌우 바퀴 속도
        # =====================================================

        v_L = (

            self.base_speed
            + correction

        )


        v_R = (

            self.base_speed
            - correction

        )


        # =====================================================
        # 속도 제한
        # =====================================================

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


        # =====================================================
        # Control output
        # =====================================================

        return np.array([
            v_L,
            v_R
        ])