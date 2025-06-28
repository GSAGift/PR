import numpy as np

def get_actual_pos_theta_by_id(data, car_id):
    # Получаем позицию тела
    x, y, _ = data.xpos[car_id]

    # Получаем ориентацию тела как матрицу 3x3
    xmat = data.xmat[car_id].reshape(3, 3)

    # Вычисляем угол поворота вокруг оси Z (для 2D)
    theta = np.arctan2(xmat[1, 0], xmat[0, 0])

    return x, y, theta

def get_target_pos_theta(target_point, x, y):
    dx = target_point[0] - x
    dy = target_point[1] - y

    dist_to_target = np.hypot(dx, dy)
    angle_target = np.arctan2(dy, dx)

    # print("x, y, target_point[0], target_point[1], dist_to_target, angle_target", x, y, target_point[0], target_point[1], dist_to_target, angle_target)

    return dist_to_target, angle_target

class PDRegulator:
    def __init__(self, Kp_lin, Kd_lin, Kp_ang, Kd_ang):
        self.Kp_lin = Kp_lin
        self.Kd_lin = Kd_lin
        self.Kp_ang = Kp_ang
        self.Kd_ang = Kd_ang

        self.prev_dist_err = 0
        self.prev_angle_err = 0

    def pd_reg(self, actual_dist, actual_angle, target_dist, target_angle, dt):
        v = self.Kp_lin * (target_dist - actual_dist) + self.Kd_lin * ((target_dist - actual_dist) - self.prev_dist_err) / dt
        delta = self.Kp_ang * (target_angle - actual_angle) + self.Kd_ang * ((target_angle - actual_angle) - self.prev_angle_err) / dt

        self.prev_dist_err = target_dist - actual_dist
        self.prev_angle_err = target_angle - actual_angle

        return v, delta
        # return 0, 0
    
