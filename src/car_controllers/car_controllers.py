import mujoco
import numpy as np

from camera import get_image_from_camera, find_displacement
from PD_regulator.PD_regulator import get_actual_pos_theta_by_id, get_target_pos_theta

def car1_conroller(model, data, target_point, desired_positions, leader_positions_history, car1_positions, controller1):
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

    return f1, a1

def car2_conroller(model, data, target_distance, max_no_qr_frames, leader_positions_history, car2_positions, controller2, renderer, previous_tracking_mode):
    body2_name = "car2"
    body2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body2_name)

    f2, a2 = 0, 0

    if body2_id == -1:
        raise ValueError(f"Car '{body2_name}' not found.")
    else:
        x2, y2, theta2 = get_actual_pos_theta_by_id(data, body2_id)
        car2_positions.append((x2, y2))

        qr_info = get_image_from_camera(data, renderer, camera="car2_front_cam")

        if qr_info and 1 == qr_info[0].data.decode('utf-8'):
            no_qr_counter = 0
            tracking_mode = "qr"
            # Дистанция и Координата смещения центра QR-кода от центра изображения по оси x
            d, x_c = find_displacement(qr_info)
            f2, a2 = controller2.pd_reg(d, x_c, target_distance, 0, model.opt.timestep)
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
    return f2, a2