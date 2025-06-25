import mujoco
import cv2
import time
from pyzbar.pyzbar import decode
import cv2 as cv

from qr_detect_and_scan import QR_handler

def get_image_from_camera(data, renderer):
    # Обновление и рендеринг камеры
    renderer.update_scene(data, camera="car1_front_cam")
    img = renderer.render()

    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    cv.imshow('test1', img_bgr)
    handler = QR_handler()
    qr_code = handler.get_qr_img(img_bgr)
    qr_code = decode(qr_code)
    
    if qr_code:
        qr_data = qr_code[0].data.decode('utf-8')  # или 'utf-8', если текст
        print(f'Считанное значение QR: {qr_data}')
    else:
        qr_text = None
        print('QR-код не найден')

    #Отображение
    cv2.imshow("Robot Camera View", img_bgr)    
    

    #Выход по ESC или по истечении времени
    if cv2.waitKey(1) == 27:  # Код клавиши ESC
        return
    return qr_code
    
    # value, size, location = find_qr(img_bgr)
    
    # if value == nash:
    #     main_car_coord = find_main_car_location(size, location)
        
        
    


    # cv2.destroyAllWindows()

def find_qr(img):
    pass

    return 0
    # find qr value and location
    
def find_main_car_location(size, location):
    pass
    # find x and y distance
    
    


    