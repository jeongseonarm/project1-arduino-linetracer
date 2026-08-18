# main.py

import numpy as np

from utils.map_generator import MapGenerator
from utils.car_model import KinematicVehicle
from utils.sensor import LineSensorArray
from utils.control import LineController
from utils.visualization import animate_vehicle


# =========================
# 맵 생성
# =========================

game_map = MapGenerator(
    x_min=-30,
    x_max=30,
    y_min=-10,
    y_max=100,
    default_reflectance=1023
)


# =========================
# 검은색 라인 생성
# =========================

for x in range(-1, 2):

    for y in range(-10, 101):

        game_map.set_reflectance(
            x=x,
            y=y,
            value=0
        )


# =========================
# 전체 맵에 랜덤 노이즈 추가
# =========================

NOISE_LEVEL = 50

for x in range(-30, 31):

    for y in range(-10, 101):

        current_value = game_map.get_reflectance(
            x,
            y
        )

        noise = np.random.randint(
            -NOISE_LEVEL,
            NOISE_LEVEL + 1
        )

        new_value = (
            current_value
            + noise
        )

        new_value = np.clip(
            new_value,
            0,
            1023
        )

        game_map.set_reflectance(
            x=x,
            y=y,
            value=new_value
        )


# =========================
# 차량 모델
# =========================

DT = 0.02

vehicle = KinematicVehicle(
    track_width=20,
    dt=DT
)


# =========================
# 센서
# =========================
S = 2.5

sensor = LineSensorArray(
    sensor_distance=10,
    sensor_spacing= S
)


# =========================
# 제어기
# =========================

controller = LineController(
    base_speed=10.0,
    kp=0.5,
    ki=0.01,
    kd=0.1,
    dt= DT,
    sensor_spacing= S,
    threshold=200.0,
    max_speed=10.0,
    min_speed=-5.0
)


# =========================
# 초기 차량 상태
# =========================

state = np.array(
    [
        2.0,
        0.0,
        np.pi / 2
    ]
)


# =========================
# 시뮬레이션
# =========================

SIMULATION_TIME = 17

steps = int(
    SIMULATION_TIME / DT
)


trajectory = [
    state.copy()
]


for _ in range(steps):

    # -------------------------
    # 센서 측정
    # -------------------------

    sensor_values = sensor.read(
        game_map,
        state
    )


    # -------------------------
    # 제어기
    # -------------------------

    control = controller.control(
        sensor_values
    )


    # -------------------------
    # 차량 업데이트
    # -------------------------

    state = vehicle.update(
        state,
        control
    )


    # -------------------------
    # trajectory 저장
    # -------------------------

    trajectory.append(
        state.copy()
    )


trajectory = np.array(
    trajectory
)


# =========================
# 시각화
# =========================

animate_vehicle(
    game_map,
    trajectory,
    DT,
    sensor
)