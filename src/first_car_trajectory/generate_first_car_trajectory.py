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


if __name__ == '__main__':
    trajectory = generate_ellipse_trajectory(a=5, b=3, num_points=20)

    for i, point in enumerate(trajectory):
        print(f"Точка {i+1}: {point}")
