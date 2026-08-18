import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def draw_vehicle(ax, state, color="red"):
    x, y, theta = state
    length = 20
    width = 15

    vehicle = Rectangle(
        (-length / 2, -width / 2),
        length,
        width,
        linewidth=2,
        edgecolor=color,
        facecolor="none"
    )

    transform = Affine2D().rotate(theta).translate(x, y) + ax.transData
    vehicle.set_transform(transform)
    ax.add_patch(vehicle)

    return vehicle


def draw_sensors(ax, sensor_positions):
    sensors = ax.scatter(
        sensor_positions[:, 0],
        sensor_positions[:, 1],
        s=40,
        c="blue",
        zorder=5
    )
    return sensors


def animate_vehicle(
    game_map,
    trajectory,
    dt,
    sensor,
    history_flower1=None,
    history_flower2=None,
    history_watering=None,
    history_mode=None
):
    fig, ax = plt.subplots(figsize=(10, 8))

    # 우측 여백을 확보하여 상태 표시 패널을 그래프 바깥으로 이동
    fig.subplots_adjust(right=0.70)

    # 맵
    ax.imshow(
        game_map.get_map(),
        cmap="gray",
        vmin=0,
        vmax=1023,
        origin="lower",
        extent=[
            game_map.x_min,
            game_map.x_max,
            game_map.y_min,
            game_map.y_max
        ]
    )

    # ============================================================
    # 화분 위치 시각화 패치 (X: 6~14, Y: 48~52 / 98~102)
    # ============================================================
    POT_X_START = 6
    POT_WIDTH = 8
    POT_HEIGHT = 4

    # 화분 1 (Y=50 근처)
    pot1_bg = Rectangle((POT_X_START, 48), POT_WIDTH, POT_HEIGHT, edgecolor="saddlebrown", facecolor="lightgray", linewidth=1.5, zorder=3)
    pot1_fill = Rectangle((POT_X_START, 48), 0, POT_HEIGHT, facecolor="dodgerblue", alpha=0.8, zorder=4)
    pot1_text = ax.text(POT_X_START + POT_WIDTH + 1, 50, "Flower 1: 0%", color="blue", fontsize=9, fontweight="bold", verticalalignment="center", zorder=5)

    # 화분 2 (Y=100 근처)
    pot2_bg = Rectangle((POT_X_START, 98), POT_WIDTH, POT_HEIGHT, edgecolor="saddlebrown", facecolor="lightgray", linewidth=1.5, zorder=3)
    pot2_fill = Rectangle((POT_X_START, 98), 0, POT_HEIGHT, facecolor="dodgerblue", alpha=0.8, zorder=4)
    pot2_text = ax.text(POT_X_START + POT_WIDTH + 1, 100, "Flower 2: 0%", color="blue", fontsize=9, fontweight="bold", verticalalignment="center", zorder=5)

    ax.add_patch(pot1_bg)
    ax.add_patch(pot1_fill)
    ax.add_patch(pot2_bg)
    ax.add_patch(pot2_fill)

    # 차량 및 센서 초기 설정
    vehicle = draw_vehicle(ax, trajectory[0])
    sensor_positions = sensor.get_sensor_positions(trajectory[0])
    sensors = draw_sensors(ax, sensor_positions)

    # ============================================================
    # 그래프 우측 외부 상태 정보 패널 (LCR, 모드, 급수 상태)
    # ============================================================
    info_text = ax.text(
        1.3, 0.95, "",
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round,pad=0.6", facecolor="whitesmoke", edgecolor="gray", alpha=0.9)
    )

    MODE_NAMES = {0: "STOP 🛑", 1: "GO 🚗", 2: "TURN 🔄"}

    def update(i):
        nonlocal vehicle, sensors

        vehicle.remove()
        sensors.remove()

        vehicle = draw_vehicle(ax, trajectory[i])
        sensor_positions = sensor.get_sensor_positions(trajectory[i])
        sensors = draw_sensors(ax, sensor_positions)
        sensor_values = sensor.read(game_map, trajectory[i])

        # 히스토리 데이터 로드
        water1 = history_flower1[i] if history_flower1 is not None else 0.0
        water2 = history_flower2[i] if history_flower2 is not None else 0.0
        is_watering = history_watering[i] if history_watering is not None else 0
        mode_val = history_mode[i] if history_mode is not None else 0

        # 화분 파란색 물 채우기 폭 및 게이지 텍스트 업데이트
        pot1_fill.set_width(POT_WIDTH * (water1 / 100.0))
        pot2_fill.set_width(POT_WIDTH * (water2 / 100.0))

        pot1_text.set_text(f"Flower 1: {water1:.0f}%")
        pot2_text.set_text(f"Flower 2: {water2:.0f}%")

        # 우측 상태 정보 패널 텍스트 업데이트
        mode_str = MODE_NAMES.get(mode_val, "UNKNOWN")
        watering_str = "ON 💧" if is_watering == 1 else "OFF ⚪"

        info_text.set_text(
            "[ System Status ]\n"
            "---------------------\n"
            f"Vehicle Mode : {mode_str}\n"
            f"Watering State: {watering_str}\n\n"
            "[ Line Sensors ]\n"
            "---------------------\n"
            f"Left   (L) : {sensor_values[0]:.0f}\n"
            f"Center (C) : {sensor_values[1]:.0f}\n"
            f"Right  (R) : {sensor_values[2]:.0f}\n\n"
            "[ Flower Water ]\n"
            "---------------------\n"
            f"Flower 1 : {water1:.0f}%\n"
            f"Flower 2 : {water2:.0f}%"
        )

        ax.set_title(f"Simulation Time = {i * dt:.2f} s")

        return vehicle, sensors, info_text, pot1_fill, pot2_fill, pot1_text, pot2_text

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=dt * 1000,
        blit=False,
        repeat=False
    )

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_xlim(game_map.x_min, game_map.x_max)
    ax.set_ylim(game_map.y_min, game_map.y_max)

    plt.show()