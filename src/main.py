import mujoco
from mujoco import viewer
import os

# --- Путь к основной сцене ---
MODEL_FILE = os.path.join(os.path.dirname(__file__), '..', 'model', 'scene.xml')


def control_func(model, data):
    """
    Основная функция управления. В данный момент пустая,
    чтобы машина оставалась неподвижной.
    """
    pass


if __name__ == '__main__':
    model = mujoco.MjModel.from_xml_path(MODEL_FILE)
    data = mujoco.MjData(model)

    mujoco.set_mjcb_control(control_func)

    # Запускаем симуляцию
    with viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
