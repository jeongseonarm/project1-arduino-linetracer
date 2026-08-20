import numpy as np
import matplotlib.pyplot as plt

from utils.map_generator import MapGenerator
from utils.car_model import KinematicVehicle
from utils.sensor import LineSensorArray
from utils.control import LineController
from utils.visualization import animate_vehicle

# ============================================================
# 맵 생성 및 라인 설정
# ============================================================
Y_MIN, Y_MAX = -10, 150
game_map = MapGenerator(x_min=-30, x_max=30, y_min=Y_MIN, y_max=Y_MAX, default_reflectance=1023)

# 중앙 검은색 라인 생성 (x = -1 ~ 1)
for x in range(-1, 2):
    for y in range(Y_MIN, Y_MAX + 1):
        game_map.set_reflectance(x=x, y=y, value=0)

# 전체 맵 노이즈 추가
NOISE_LEVEL = 50
for x in range(-30, 31):
    for y in range(Y_MIN, Y_MAX + 1):
        current_val = game_map.get_reflectance(x, y)
        noise = np.random.randint(-NOISE_LEVEL, NOISE_LEVEL + 1)
        game_map.set_reflectance(x=x, y=y, value=int(np.clip(current_val + noise, 0, 1023)))

# ============================================================
# 차량 / 센서 / 제어기 설정
# ============================================================
DT = 0.02
S = 2.5

vehicle = KinematicVehicle(track_width=20, dt=DT)
sensor = LineSensorArray(sensor_distance=10, sensor_spacing=S)
controller = LineController(
    base_speed=10.0, kp=0.5, ki=0.0, kd=0.1, dt=DT,
    sensor_spacing=S, threshold=200.0, max_speed=10.0, min_speed=-5.0
)

# 초기 차량 상태 [x, y, heading]
state = np.array([2.0, 0.0, np.pi / 2])

# ============================================================
# FSM 상태 및 화분 위치 정의
# ============================================================
STOP, GO, TURN = 0, 1, 2
vehicle_mode = STOP

FLOWER1_Y = 50.0   # 중간 화분 위치
FLOWER2_Y = 100.0  # 끝 화분 위치
OBSTACLE_Y = 10.0  # 복귀 시 장애물 위치

# 화분 수분 상태 (0.0: DRY ~ 1.0: WET / 100%)
flower1_water = 0.0
flower2_water = 0.0
flower1_status = 0 # 0: DRY, 1: WET
flower2_status = 0 # 0: DRY, 1: WET

watering = 0       # 0: OFF, 1: ON
watering_count = 0

STOP_TIME = 0.5
stop_start_time = None

TURN_SPEED = 5.0
TURN_DURATION = 6.28
turn_start_time = None

# ============================================================
# 데이터 기록용 배열 (Plot용)
# ============================================================
SIMULATION_TIME = 35.0
steps = int(SIMULATION_TIME / DT)

trajectory = [state.copy()]
history_time = []
history_flower1 = []
history_flower2 = []
history_watering = []
history_mode = []

# ============================================================
# 메인 시뮬레이션 루프
# ============================================================
for step in range(steps):
    current_time = step * DT
    y_pos = state[1]

    # 1. 센서 기반 화분 및 장애물 감지 논리 (가상 센서)
    flower_detected = False
    object_detected = False

    # 정방향 진행 중 화분 감지
    if vehicle_mode == GO and state[2] > 0: # 북쪽 향해 진행 중
        if flower1_status == 0 and y_pos >= FLOWER1_Y:
            flower_detected = True
        elif flower1_status == 1 and flower2_status == 0 and y_pos >= FLOWER2_Y:
            flower_detected = True

    # 180도 회전 후 복귀 중 장애물 감지
    elif vehicle_mode == GO and state[2] < 0: # 남쪽 향해 복귀 중
        if watering_count >= 2 and y_pos <= OBSTACLE_Y:
            object_detected = True

    # 2. FSM 상태 변경 및 제어
    control = np.array([0.0, 0.0])

    # [t1 -> t2] 초기 대기 후 출발
    if step == 0:
        vehicle_mode = GO
        print(f"t2: 화분 DRY -> 차량 GO")

    # GO 상태 처리
    if vehicle_mode == GO:
        sensor_values = sensor.read(game_map, state)
        control = controller.control(sensor_values)

        if flower_detected:
            vehicle_mode = STOP
            stop_start_time = current_time
            control = np.array([0.0, 0.0])
            print(f"[{current_time:.1f}s] 화분 인지 -> 차량 STOP")

        elif object_detected:
            vehicle_mode = STOP
            control = np.array([0.0, 0.0])
            print(f"[{current_time:.1f}s] 장애물 인지 -> 차량 정지 (시뮬레이션 완료)")

    # STOP 상태 및 급수 제어
    elif vehicle_mode == STOP:
        control = np.array([0.0, 0.0])

        # 정지 후 일정 시간이 지나면 급수 시작 (t4, t8)
        if stop_start_time is not None and (current_time - stop_start_time >= STOP_TIME):
            if (flower1_status == 0 or flower2_status == 0) and watering == 0:
                watering = 1
                print(f"[{current_time:.1f}s] 차량 STOP 완료 -> 급수 시작 (WATERING ON)")

        # 급수 진행 중 (화분 물 채우기)
        if watering == 1:
            WATER_RATE = 0.4  # 초당 충전률 (약 2.5초 소요)
            if flower1_status == 0:
                flower1_water = min(1.0, flower1_water + WATER_RATE * DT)
                if flower1_water >= 1.0:
                    flower1_status = 1
                    watering = 0
                    watering_count += 1
                    vehicle_mode = GO
                    stop_start_time = None
                    print(f"[{current_time:.1f}s] t5/t6: 화분1 WET 완료 -> 급수 STOP -> 차량 GO")

            elif flower1_status == 1 and flower2_status == 0:
                flower2_water = min(1.0, flower2_water + WATER_RATE * DT)
                if flower2_water >= 1.0:
                    flower2_status = 1
                    watering = 0
                    watering_count += 1
                    vehicle_mode = TURN
                    turn_start_time = current_time
                    stop_start_time = None
                    print(f"[{current_time:.1f}s] t9/t10: 화분2 WET 완료 -> 급수 STOP -> 차량 TURN")

    # TURN 상태 (180도 제자리 회전)
    elif vehicle_mode == TURN:
        control = np.array([-TURN_SPEED, TURN_SPEED])
        if current_time - turn_start_time >= TURN_DURATION:
            vehicle_mode = GO  # 회전 완료 후 복귀 진행 (t11 -> t12)
            print(f"[{current_time:.1f}s] t11/t12: 회전 완료 -> 차량 GO (복귀)")

    # 차량 위치 업데이트
    state = vehicle.update(state, control)
    trajectory.append(state.copy())

    # 시각화용 히스토리 저장
    history_time.append(current_time)
    history_flower1.append(flower1_water * 100) # 퍼센트
    history_flower2.append(flower2_water * 100)
    history_watering.append(watering)
    history_mode.append(vehicle_mode)

trajectory = np.array(trajectory)

# ============================================================

# 맵 애니메이션 실행
animate_vehicle(
    game_map,
    trajectory,
    DT,
    sensor,
    history_flower1=history_flower1,
    history_flower2=history_flower2,
    history_watering=history_watering
)