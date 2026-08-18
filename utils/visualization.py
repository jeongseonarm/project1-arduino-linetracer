# utils/visualization.py

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def draw_vehicle(
        ax,
        state,
        color="red"
):

    x, y, theta = state

    length = 20
    width = 15


    # =========================
    # 차량 생성
    # =========================

    vehicle = Rectangle(
        (-length / 2, -width / 2),
        length,
        width,
        linewidth=2,
        edgecolor=color,
        facecolor="none"
    )


    # =========================
    # 차량 위치 및 방향 적용
    # =========================

    transform = (
        Affine2D()
        .rotate(theta)
        .translate(x, y)
        + ax.transData
    )


    vehicle.set_transform(
        transform
    )


    ax.add_patch(
        vehicle
    )


    return vehicle


def draw_sensors(
        ax,
        sensor_positions
):

    # =========================
    # 센서 위치
    # =========================

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
        sensor
):

    # =========================
    # Figure
    # =========================

    fig, ax = plt.subplots(
        figsize=(8, 8)
    )


    # =========================
    # 맵
    # =========================

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


    # =========================
    # 차량 초기 위치
    # =========================

    vehicle = draw_vehicle(
        ax,
        trajectory[0]
    )


    # =========================
    # 센서 초기 위치
    # =========================

    sensor_positions = sensor.get_sensor_positions(
        trajectory[0]
    )

    sensors = draw_sensors(
        ax,
        sensor_positions
    )


    # =========================
    # 센서값 초기값
    # =========================

    sensor_values = sensor.read(
        game_map,
        trajectory[0]
    )


    # =========================
    # 고정 센서 정보 표시
    # =========================

    sensor_text = ax.text(
        0.02,
        0.95,
        "",
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top"
    )


    # =========================
    # Animation update
    # =========================

    def update(i):

        nonlocal vehicle
        nonlocal sensors


        # -------------------------
        # 기존 차량 삭제
        # -------------------------

        vehicle.remove()


        # -------------------------
        # 기존 센서 삭제
        # -------------------------

        sensors.remove()


        # -------------------------
        # 차량 업데이트
        # -------------------------

        vehicle = draw_vehicle(
            ax,
            trajectory[i]
        )


        # -------------------------
        # 센서 위치 계산
        # -------------------------

        sensor_positions = sensor.get_sensor_positions(
            trajectory[i]
        )


        # -------------------------
        # 센서 업데이트
        # -------------------------

        sensors = draw_sensors(
            ax,
            sensor_positions
        )


        # -------------------------
        # 센서값 읽기
        # -------------------------

        sensor_values = sensor.read(
            game_map,
            trajectory[i]
        )


        # -------------------------
        # 센서값 표시
        # -------------------------

        sensor_text.set_text(
            f"L: {sensor_values[0]:.0f}\n"
            f"C: {sensor_values[1]:.0f}\n"
            f"R: {sensor_values[2]:.0f}"
        )


        # -------------------------
        # 시간
        # -------------------------

        ax.set_title(
            f"Time = {i * dt:.2f} s"
        )


        return (
            vehicle,
            sensors,
            sensor_text
        )


    # =========================
    # Animation
    # =========================

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=dt * 1000,
        blit=False,
        repeat=False
    )


    # =========================
    # 축
    # =========================

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")


    ax.set_xlim(
        game_map.x_min,
        game_map.x_max
    )


    ax.set_ylim(
        game_map.y_min,
        game_map.y_max
    )


    plt.show()