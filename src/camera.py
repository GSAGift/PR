import mujoco
import cv2
import time

def get_image_from_camera(data, renderer):
    # Обновление и рендеринг камеры
    renderer.update_scene(data, camera="car1_front_cam")
    img = renderer.render()

    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
        # Отображение
    cv2.imshow("Robot Camera View", img_bgr)    
    

    #Выход по ESC или по истечении времени
    if cv2.waitKey(1) == 27:  # Код клавиши ESC
        return

    # cv2.destroyAllWindows()
    