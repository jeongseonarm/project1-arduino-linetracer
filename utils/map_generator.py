import numpy as np


class MapGenerator:
    def __init__(
        self,
        x_min,
        x_max,
        y_min,
        y_max,
        default_reflectance=1023
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.width = x_max - x_min + 1
        self.height = y_max - y_min + 1

        # grid[y][x] 형태
        self.grid = np.full(
            (self.height, self.width),
            default_reflectance,
            dtype=int
        )

    def _to_index(self, x, y):
        """
        실제 좌표를 numpy 배열 인덱스로 변환

        예:
        (-30, -10) -> (0, 0)
        (0, 0)     -> (30, 10)
        (30, 50)   -> (60, 60)
        """

        index_x = x - self.x_min
        index_y = y - self.y_min

        return index_x, index_y

    def set_reflectance(self, x, y, value):
        """특정 좌표의 반사율 설정"""

        index_x, index_y = self._to_index(x, y)

        if (
            0 <= index_x < self.width
            and 0 <= index_y < self.height
        ):
            self.grid[index_y][index_x] = max(
                0,
                min(1023, value)
            )

    def get_reflectance(self, x, y):
        """특정 좌표의 반사율 반환"""

        index_x, index_y = self._to_index(
            int(round(x)),
            int(round(y))
        )

        if (
            0 <= index_x < self.width
            and 0 <= index_y < self.height
        ):
            return self.grid[index_y][index_x]

        # 맵 밖이면 흰색 처리
        return 1023

    def get_map(self):
        return self.grid