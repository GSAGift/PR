import mujoco
from mujoco import viewer
import numpy as np

from first_car_trajectory.generate_first_car_trajectory import generate_ellipse_trajectory
from PD_regulator.PD_regulator import PDRegulator, get_actual_pos_theta_by_id, get_target_pos_theta

model_pass = "/Users/polinakuranova/uni/robot_programming_course/course_work_2/PR/model/scene.xml"

def control_func(model, data):
    body_name="car1"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    
    if body_id == -1:
        raise ValueError(f"Тело '{body_name}' не найдено в модели.")
    else:
        # print(f"ID тела '{body_name}': {body_id}")
        # position = data.xpos[body_id]  # world position of the body
        # print("\nПозиция тела в мировых координатах:", position)

        x, y, theta = get_actual_pos_theta_by_id(data, body_id)

        print(x, y, theta)

        # Получаем требуемые положение и ориентацию
        dist_to_target, angle_target = get_target_pos_theta(trajectory, x, y)

        data.ctrl = controller.pd_reg(dist_to_target, angle_target, np.hypot(x, y), theta, model.opt.timestep)


if __name__ == '__main__':
    trajectory = generate_ellipse_trajectory(a=1, b=0.5, num_points=100, cx=0, cy=0)

    print(trajectory)

    controller = PDRegulator(Kp_lin = 0.5, Kd_lin = 0.5, Kp_ang = -0.01, Kd_ang = -0.3)

    # Загрузка модели
    model = mujoco.MjModel.from_xml_path(model_pass)
    data = mujoco.MjData(model)

    mujoco.set_mjcb_control(control_func)

    # Запуск интерактивного окна
    with viewer.launch(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
