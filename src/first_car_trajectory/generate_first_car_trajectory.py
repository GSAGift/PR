import numpy as np

def generate_ellipse_trajectory(a, b, num_points=100, cx=0, cy=0):
    """
    Генерирует траекторию точек на эллипсе.
    
    Параметры:
    a (float): длина большой полуоси
    b (float): длина малой полуоси
    num_points (int): количество точек для генерации
    cx (float): x-координата центра эллипса
    cy (float): y-координата центра эллипса
    
    Возвращает:
    list of tuples: список пар (x, y), представляющих точки на эллипсе
    """
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = cx + a * np.cos(theta)
    y = cy + b * np.sin(theta)
    return list(zip(x, y))

def infinite_trajectory_generator(a=1.0, b=1.0, center=(0.0, 0.0), angular_speed=0.01):
    """
    Бесконечный генератор точек на эллипсе (или окружности).
    
    Параметры:
    a (float): большая полуось (по X)
    b (float): малая полуось (по Y)
    center (tuple): центр траектории (cx, cy)
    angular_speed (float): угловая скорость (рад/шаг)
    
    Yields:
    tuple: (x, y) — координаты следующей точки траектории
    """
    cx, cy = center
    theta = 0.0  # Начальный угол
    while True:
        x = cx + a * np.cos(theta)
        y = cy + b * np.sin(theta)
        # print("actual x, y, theta", x, y, theta)
        yield x, y
        theta += angular_speed
        # Ограничиваем theta, чтобы избежать больших чисел
        if theta > 2 * np.pi:
            theta -= 2 * np.pi


def infinite_sin_trajectory_generator(amplitude=1.0, frequency=1.0, speed=0.1,
                                  offset=0.0, phase=0.0, direction='x'):
    """
    Бесконечный генератор точек по синусоидальной траектории.

    Параметры:
    amplitude (float): Амплитуда синусоиды
    frequency (float): Частота (количество волн на единицу длины)
    speed (float): Скорость движения вдоль основной оси (рад/шаг)
    offset (float): Вертикальный сдвиг графика
    phase (float): Начальная фаза синусоиды
    direction (str): 'x' — синусоида вдоль оси X, 'y' — вдоль Y

    Yields:
    tuple: (x, y) — координаты следующей точки траектории
    """
    t = 0.0  # параметр, который будет расти со временем
    while True:
        if direction == 'x':
            x = t
            y = amplitude * np.sin(frequency * t + phase) + offset
        elif direction == 'y':
            y = t
            x = amplitude * np.sin(frequency * y + phase) + offset
        else:
            raise ValueError("direction должен быть 'x' или 'y'")

        yield x, y
        t += speed

class TargetPoint:
    def __init__(self, gen):
        self.x = 0
        self.y = 0
        self.gen = gen
        self.x, self.y = next(self.gen)

    def next(self):
        self.x, self.y = next(self.gen)

        return self.x, self.y

if __name__ == '__main__':
    gen = infinite_trajectory_generator(a=1, b=0.5, center=(0, 0), angular_speed=0.1)

    # Получаем первые 10 точек
    for i in range(10):
        point = next(gen)
        print(f"Точка {i+1}: {point}")
