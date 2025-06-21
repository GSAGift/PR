import mujoco
from mujoco import viewer
import numpy as np

model_pass = "/Users/polinakuranova/uni/robot_programming_course/course_work_2/PR/model/scene.xml"

def control_func(model, data):
    pass

if __name__ == '__main__':
    # Загрузка модели
    model = mujoco.MjModel.from_xml_path(model_pass)
    data = mujoco.MjData(model)

    mujoco.set_mjcb_control(control_func)

    # Запуск интерактивного окна
    with viewer.launch(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
