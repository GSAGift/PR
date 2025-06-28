import math
import mujoco
import cv2
import time
from pyzbar.pyzbar import decode
import cv2 as cv

from qr_detect_and_scan import QR_handler

def get_image_from_camera(data, renderer, camera):
    # Обновление и рендеринг камеры
    renderer.update_scene(data, camera=camera)
    img = renderer.render()

    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    print(img_bgr.shape)
    qr_info = decode(img_bgr)
    
    if qr_info:
        qr_data = qr_info[0].data.decode('utf-8')  # или 'utf-8', если текст
        print(f'Считанное значение QR: {qr_data}')
    else:
        qr_text = None
        print('QR-код не найден')
        
    #Отображение
    cv2.imshow("Robot Camera View", img_bgr)    

    #Выход по ESC или по истечении времени
    if cv2.waitKey(1) == 27:  # Код клавиши ESC
        return
    return qr_info
    

def find_displacement(qr_info):
    
    # Ширина QR-кода на ИЗОБРАЖЕНИИ
    width = qr_info[0].rect.width
    
    # Фокусное расстояние линзы камеры
    f =  236 / (2 * math.tan(45 / 2))
    # Подсчитанное расстояние от камеры до QR-кода
    d = (8 * f)/width 
    

    # Координата центра ИЗОБРАЖЕНИЯ
    x_centered = 118
    
    #Координаты первого угла кода
    x1 = qr_info[0].polygon[0].x
    y1 = qr_info[0].polygon[0].y
    
    #Координаты третьего угла кода
    x3 = qr_info[0].polygon[2].x
    y3 = qr_info[0].polygon[2].y
    
    # Координата смещения центра QR-кода от центра изображения по оси x
    x_c = ((x1 + x3)/2) - x_centered
    
    return d, x_c
    
    
    
    


    