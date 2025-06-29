import mujoco
from mujoco import viewer
import numpy as np
import matplotlib.pyplot as plt

from first_car_trajectory.generate_first_car_trajectory import infinite_trajectory_generator, TargetPoint
from PD_regulator.PD_regulator import PDRegulator
from car_controllers.car_controllers import car1_conroller, car2_conroller

model_pass = "../model/scene.xml"

# --- Для записи траекторий ---
car1_positions = []
car2_positions = []
desired_positions = []

leader_positions_history = []
tracking_mode = "history"
no_qr_counter = 0
max_no_qr_frames = 10  # например, 10 шагов без QR — меняем режим
previous_tracking_mode = tracking_mode

def control_func(model, data):
    global car1_positions, car2_positions, desired_positions, tracking_mode, no_qr_counter, max_no_qr_frames, previous_tracking_mode

    f1, a1 = car1_conroller(model, data, target_point, desired_positions, leader_positions_history, car1_positions, controller1)
    f2, a2 = car2_conroller(model, data, target_distance, max_no_qr_frames, leader_positions_history, car2_positions, controller2, renderer, previous_tracking_mode)

    data.ctrl = [f1, a1, f2, a2]


if __name__ == '__main__':
    target_points = []
    try:
        gen = infinite_trajectory_generator(a=1, b=0.5, center=(0, 0), angular_speed=-0.1)

        target_point = TargetPoint(gen)
        target_points.append(target_point)
        target_distance = 0.1

        controller1 = PDRegulator(Kp_lin = -0.5, Kd_lin = 0.1, Kp_ang = 1, Kd_ang = 0.5)
        controller2 = PDRegulator(Kp_lin = -0.8, Kd_lin = 0.1, Kp_ang = -0.4, Kd_ang = -0.001)

        # Загрузка модели
        model = mujoco.MjModel.from_xml_path(model_pass)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, 320, 240)

        mujoco.set_mjcb_control(control_func)

        # Запуск интерактивного окна
        with viewer.launch(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()

    except Exception as e:
        print(f"[Ошибка] {e}")
    finally:
        if car1_positions or car2_positions or desired_positions:
            plt.figure(figsize=(10, 8))

            # Реальные траектории
            if car1_positions:
                car1_x, car1_y = zip(*car1_positions)
                plt.plot(car1_x, car1_y, label='Car 1 Actual', color='blue')

            if car2_positions:
                car2_x, car2_y = zip(*car2_positions)
                plt.plot(car2_x, car2_y, label='Car 2 Actual', color='orange')

            # Желаемые точки (те, что использовались в симуляции)
            if desired_positions:
                desired_x, desired_y = zip(*desired_positions)
                plt.plot(desired_x, desired_y, label='Desired Trajectory (used)', color='green', linestyle='--',
                         linewidth=2)

            # Опционально: полная желаемая траектория (для сравнения)
            num_preview_points = 100
            debug_gen = infinite_trajectory_generator(a=1, b=0.5, center=(0, 0), angular_speed=0.1)
            preview_desired = [next(debug_gen) for _ in range(num_preview_points)]
            px, py = zip(*preview_desired)
            plt.plot(px, py, label='Full Desired Ellipse', color='purple', linestyle=':', linewidth=2)

            plt.title("Trajectories of Cars and Desired Path")
            plt.xlabel("X Position")
            plt.ylabel("Y Position")
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            plt.tight_layout()
            plt.show(block=True)

        else:
            print("[Info] Нет данных для отображения траекторий.")