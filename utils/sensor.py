import numpy as np


class LineSensorArray:

    def __init__(
            self,
            sensor_distance,
            sensor_spacing
    ):

        # 차량 중심에서 센서까지의 앞쪽 거리
        self.sensor_distance = sensor_distance

        # 중앙 센서에서 좌/우 센서까지의 거리
        self.sensor_spacing = sensor_spacing


    def get_sensor_positions(
            self,
            state
    ):

        x, y, theta = state


        # =========================
        # 진행 방향
        # =========================

        forward_x = np.cos(theta)
        forward_y = np.sin(theta)


        # =========================
        # 차량 왼쪽 방향
        # =========================

        side_x = -np.sin(theta)
        side_y = np.cos(theta)


        # =========================
        # 센서 위치
        # =========================

        left = np.array(
            [
                x
                + self.sensor_distance * forward_x
                + self.sensor_spacing * side_x,

                y
                + self.sensor_distance * forward_y
                + self.sensor_spacing * side_y
            ]
        )


        center = np.array(
            [
                x
                + self.sensor_distance * forward_x,

                y
                + self.sensor_distance * forward_y
            ]
        )


        right = np.array(
            [
                x
                + self.sensor_distance * forward_x
                - self.sensor_spacing * side_x,

                y
                + self.sensor_distance * forward_y
                - self.sensor_spacing * side_y
            ]
        )


        return np.array(
            [
                left,
                center,
                right
            ]
        )


    def read(
            self,
            game_map,
            state
    ):

        sensor_positions = self.get_sensor_positions(
            state
        )


        sensor_values = []


        # =========================
        # 각 센서의 반사율 측정
        # =========================

        for position in sensor_positions:

            sensor_x = int(
                round(position[0])
            )

            sensor_y = int(
                round(position[1])
            )


            # 맵 밖이면 흰색으로 처리

            if (
                sensor_x < game_map.x_min
                or sensor_x > game_map.x_max
                or sensor_y < game_map.y_min
                or sensor_y > game_map.y_max
            ):

                reflectance = 1023

            else:

                reflectance = game_map.get_reflectance(
                    sensor_x,
                    sensor_y
                )


            sensor_values.append(
                reflectance
            )


        return np.array(
            sensor_values
        )