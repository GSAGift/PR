import mujoco
from mujoco import viewer
import numpy as np
import matplotlib.pyplot as plt
from camera import get_image_from_camera, find_displacement

from first_car_trajectory.generate_first_car_trajectory import infinite_trajectory_generator, TargetPoint
from PD_regulator.PD_regulator import PDRegulator, get_actual_pos_theta_by_id, get_target_pos_theta

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

    # ------ First car ------
    target_point_list = (target_point.x, target_point.y)
    desired_positions.append(target_point_list)  # Сохраняем желаемую точку

    body1_name = "car1"
    body1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body1_name)

    f1, a1 = 0, 0

    if body1_id == -1:
        raise ValueError(f"Car '{body1_name}' not found.")
    else:
        x1, y1, theta1 = get_actual_pos_theta_by_id(data, body1_id)
        leader_positions_history.append((x1, y1))

        max_history_length = 50
        if len(leader_positions_history) > max_history_length:
            leader_positions_history.pop(0)

        car1_positions.append((x1, y1))

        dist_to_target, angle_target = get_target_pos_theta(target_point_list, x1, y1)

        if dist_to_target < 0.01:

            target_point_list = target_point.next()
        f1, a1 = controller1.pd_reg(np.hypot(x1, y1), theta1, dist_to_target, angle_target, model.opt.timestep)

    # ------ Second car ------
    body2_name = "car2"
    body2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body2_name)

    f2, a2 = 0, 0

    if body2_id == -1:
        raise ValueError(f"Car '{body2_name}' not found.")
    else:
        x2, y2, theta2 = get_actual_pos_theta_by_id(data, body2_id)
        car2_positions.append((x2, y2))

        qr_info = get_image_from_camera(data, renderer, camera="car2_front_cam")

        if qr_info:
            no_qr_counter = 0
            tracking_mode = "qr"
            # Дистанция и Координата смещения центра QR-кода от центра изображения по оси x
            d, x_c = find_displacement(qr_info)
            f2, a2 = controller2.pd_reg(d, x_c, target_distance, 0, model.opt.timestep)
            # print(f2, a2)
        else:
            no_qr_counter += 1
            if no_qr_counter > max_no_qr_frames:
                tracking_mode = "history"

        if tracking_mode == "history":
            if leader_positions_history:
                history_index = min(10, len(leader_positions_history) - 1)
                target_x, target_y = leader_positions_history[-history_index]
                dx = target_x - x2
                dy = target_y - y2
                dist_to_target = np.hypot(dx, dy)
                angle_to_leader = np.arctan2(dy, dx)
                angle_error = angle_to_leader - theta2
                angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi  # нормализация

                # Передаём в регулятор
                f2, a2 = controller2.pd_reg(dist_to_target, angle_error, target_distance, 0, model.opt.timestep)

                # Сброс ошибок регулятора при смене режима
                if previous_tracking_mode != tracking_mode:
                    controller2.prev_dist_err = 0
                    controller2.prev_angle_err = 0
            else:
                f2, a2 = 0.0, 0.0
    # ------ set control ------
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
            """for i in range(len(px)):
                print(f' Target: {target_points[i].x}, {target_points[i].y}')
                print(f' Elips: {px[i]}, {py[i]}')"""


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
