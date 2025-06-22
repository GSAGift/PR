from pyzbar.pyzbar import decode
from PIL import Image

def scan_qr_code(image_path):
    """
    Сканирует QR-код на изображении и возвращает данные.
    """
    try:
        img = Image.open(image_path)
        decoded_objects = decode(img)
        if decoded_objects:
            return decoded_objects[0].data.decode('utf-8')
        else:
            return None
    except FileNotFoundError:
        return "Ошибка: Файл не найден."
    except Exception as e:
        return f"Ошибка при сканировании: {e}"

# Пример использования
image_file = "/home/amir/Studying/PR/qr_codes/qr_1.png"
qr_data = scan_qr_code(image_file)

if qr_data:
    print(f"Данные из QR-кода: {qr_data}")
else:
    print("QR-код не найден или произошла ошибка.")