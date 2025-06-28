import mujoco
from mujoco import viewer
import numpy as np
from camera import get_image_from_camera, find_displacement

from first_car_trajectory.generate_first_car_trajectory import infinite_trajectory_generator, TargetPoint
from PD_regulator.PD_regulator import PDRegulator, get_actual_pos_theta_by_id, get_target_pos_theta

model_pass="/home/amir/Studying/PR/model/scene.xml"

def control_func(model, data):
    # ------ First car ------
    target_point_list = (target_point.x, target_point.y)

    body1_name="car1"
    body1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body1_name)
    
    f1, a1 = 0, 0

    if body1_id == -1:
        raise ValueError(f"Car '{body1_name}' not found.")
    else:
        x1, y1, theta1 = get_actual_pos_theta_by_id(data, body1_id)

        # Получаем требуемые положение и ориентацию
        dist_to_target, angle_target = get_target_pos_theta(target_point_list, x1, y1)

        if dist_to_target < 0.01:
            target_point_list = target_point.next()

        f1, a1 = controller1.pd_reg(np.hypot(x1, y1), theta1, dist_to_target, angle_target, model.opt.timestep)
        
        # print(model.opt.timestep)

    # ------ Second car ------
    
    body2_name="car2"
    body2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body2_name)
    
    f2, a2 = 0, 0
    
    if body2_id == -1:
        raise ValueError(f"Car '{body2_name}' not found.")
    else:
        qr_info = get_image_from_camera(data, renderer, camera="car2_front_cam")
        
        if (qr_info):
            d, x_c = find_displacement(qr_info)
        
            f2, a2 = controller2.pd_reg(d, x_c, target_distance, 0, model.opt.timestep)
            
        else:
            x2, y2, theta2 = get_actual_pos_theta_by_id(data, body1_id)
            # d, x_c = 0.1, 0
            # f2, a2 = 1, 0
            # dist_to_target2, angle_target2 = get_target_pos_theta((x1, y1), x2, y2)
            f2, a2 = controller2.pd_reg(x1 - x2, theta1 - theta2, target_distance, 0, model.opt.timestep)
            
    # ------ set control ------
    
    data.ctrl = [f1, a1, f2, a2]



if __name__ == '__main__':
    try:
        gen = infinite_trajectory_generator(a=1, b=0.5, center=(0, 0), angular_speed=-0.1)
        target_point = TargetPoint(gen)
        
        target_distance = 1

        controller1 = PDRegulator(Kp_lin = 0.1, Kd_lin = 0.1, Kp_ang = -0.01, Kd_ang = -0.3)
        controller2 = PDRegulator(Kp_lin = 0.15, Kd_lin = 0.15, Kp_ang = 0.1, Kd_ang = 0.1)

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