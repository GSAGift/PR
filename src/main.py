import mujoco
from mujoco import viewer
import numpy as np

from first_car_trajectory.generate_first_car_trajectory import infinite_trajectory_generator, TargetPoint
from PD_regulator.PD_regulator import PDRegulator, get_actual_pos_theta_by_id, get_target_pos_theta

model_pass = "/Users/polinakuranova/uni/robot_programming_course/course_work_2/PR/model/scene.xml"

def control_func(model, data):
    # First car
    target_point_list = (target_point.x, target_point.y)

    body_name="car1"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    
    if body_id == -1:
        raise ValueError(f"Car '{body_name}' not found.")
    else:
        x, y, theta = get_actual_pos_theta_by_id(data, body_id)

        # Получаем требуемые положение и ориентацию
        dist_to_target, angle_target = get_target_pos_theta(target_point_list, x, y)
        # print(dist_to_target, angle_target)
        if dist_to_target < 0.01:
            target_point_list = target_point.next()

        data.ctrl = controller.pd_reg(dist_to_target, angle_target, np.hypot(x, y), theta, model.opt.timestep)

    # Other car
    # pass


if __name__ == '__main__':
    try:
        gen = infinite_trajectory_generator(a=1, b=0.5, center=(0, 0), angular_speed=-0.1)
        target_point = TargetPoint(gen)

        controller = PDRegulator(Kp_lin = 1, Kd_lin = 1, Kp_ang = -0.01, Kd_ang = -0.3)

        # Загрузка модели
        model = mujoco.MjModel.from_xml_path(model_pass)
        data = mujoco.MjData(model)

        mujoco.set_mjcb_control(control_func)

        # Запуск интерактивного окна
        with viewer.launch(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
    except Exception as e:
        print(f"[Ошибка] {e}")