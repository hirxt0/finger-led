"""ООП пример для finger-led

Отличие ООП-решения от процедурного кода:
- Инкапсуляция: данные и логика объединены в классы (SerialLEDController, FingerDetector)
- Переиспользование: каждый класс можно легко использовать в других проектах
- Читаемость: код структурирован и понятнее за счёт разделения ответственности
- Расширяемость: легко добавлять новые методы и функциональность
"""

import argparse
import time
from collections import deque, Counter
import cv2
import mediapipe as mp

try:
    import serial
    from serial.serialutil import SerialException
except Exception:
    serial = None
    SerialException = Exception


class SerialLEDController:
    """Класс для управления LED через serial-порт"""
    
    def __init__(self, port, baudrate, simulate=False):
        self.port = port
        self.baudrate = baudrate
        self.simulate = simulate
        self.ser = None
        self.last_sent = None
        
        if not self.simulate:
            self._connect()
    
    def _connect(self):
        """Подключение к serial-порту"""
        if serial is None:
            raise SystemExit("Модуль serial не установлен")
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2.0)
            print(f"[Serial] Открыт порт {self.port}, {self.baudrate}")
        except SerialException as e:
            raise SystemExit(f"Ошибка подключения к порту: {e}")
    
    def send_finger_count(self, count):
        """Отправка количества пальцев на Arduino"""
        if count == self.last_sent:
            return
        
        self.last_sent = count
        
        if self.ser:
            msg = f"{count}\n"
            try:
                self.ser.write(msg.encode('utf-8'))
            except SerialException as e:
                print(f"Ошибка при записи: {e}")
                self.ser = None
        else:
            print(f"[SIM] -> {count}")
    
    def close(self):
        """Закрытие соединения"""
        if self.ser:
            self.ser.close()


class FingerDetector:
    """Класс для обнаружения пальцев с помощью MediaPipe"""
    
    TIP_IDS = [4, 8, 12, 16, 20]
    PIP_IDS = [3, 6, 10, 14, 18]
    
    def __init__(self, max_num_hands=1, min_detection_confidence=0.6, min_tracking_confidence=0.5):
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            model_complexity=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
    
    def detect_fingers(self, frame):
        """Определение количества поднятых пальцев"""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        
        finger_count = 0
        finger_states = None
        handedness_label = None
        hand_landmarks = None
        
        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]
            handedness_label = result.multi_handedness[0].classification[0].label
            landmarks = hand_landmarks.landmark
            finger_states = self._fingers_up_from_landmarks(landmarks, handedness_label)
            finger_count = sum(1 for f in finger_states if f)
        
        return finger_count, finger_states, handedness_label, hand_landmarks
    
    def _fingers_up_from_landmarks(self, landmarks, handedness_label):
        """Определение какие пальцы подняты"""
        fingers = [False] * 5
        
        # Большой палец (проверка по оси X)
        if handedness_label == 'Right':
            fingers[0] = landmarks[self.TIP_IDS[0]].x < landmarks[self.PIP_IDS[0]].x
        else:
            fingers[0] = landmarks[self.TIP_IDS[0]].x > landmarks[self.PIP_IDS[0]].x
        
        # Остальные пальцы (проверка по оси Y)
        for i in range(1, 5):
            tip_id = self.TIP_IDS[i]
            pip_id = self.PIP_IDS[i]
            fingers[i] = landmarks[tip_id].y < landmarks[pip_id].y
        
        return fingers
    
    def draw_landmarks(self, frame, hand_landmarks):
        """Отрисовка landmarks на кадре"""
        self.mp_drawing.draw_landmarks(
            frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
        )
    
    def close(self):
        """Освобождение ресурсов"""
        self.hands.close()


class FingerLEDApp:
    """Основной класс приложения"""
    
    def __init__(self, args):
        self.args = args
        self.led_controller = SerialLEDController(
            args.port, args.baudrate, args.simulate
        )
        self.finger_detector = FingerDetector()
        self.cap = None
        self.recent = deque(maxlen=args.smoothing)
    
    def setup_camera(self):
        """Инициализация камеры"""
        self.cap = cv2.VideoCapture(self.args.camera)
        if not self.cap.isOpened():
            raise SystemExit("Не удалось открыть камеру")
        print("Нажмите ESC чтобы выйти")
    
    def process_frame(self, frame):
        """Обработка одного кадра"""
        if self.args.resize_width:
            h, w = frame.shape[:2]
            scale = self.args.resize_width / float(w)
            frame = cv2.resize(frame, (self.args.resize_width, int(h * scale)))
        
        finger_count, finger_states, handedness_label, hand_landmarks = \
            self.finger_detector.detect_fingers(frame)
        
        # Рисуем landmarks если рука обнаружена
        if hand_landmarks:
            self.finger_detector.draw_landmarks(frame, hand_landmarks)
            cv2.putText(frame, f"{handedness_label}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
        
        # Сглаживание результатов
        self.recent.append(finger_count)
        smoothed = Counter(self.recent).most_common(1)[0][0]
        
        # Отправка данных на Arduino
        self.led_controller.send_finger_count(smoothed)
        
        # Отображение информации на кадре
        cv2.putText(frame, f"Fingers: {finger_count} (smoothed: {smoothed})",
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        if finger_states is not None:
            txt = "".join(['1' if x else '0' for x in finger_states])
            cv2.putText(frame, txt, (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return frame
    
    def run(self):
        """Основной цикл приложения"""
        self.setup_camera()
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Кадры не читаются")
                    break
                
                frame = self.process_frame(frame)
                cv2.imshow("Finger -> Arduino", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC
                    break
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Очистка ресурсов"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.finger_detector.close()
        self.led_controller.close()


def main(args):
    """Точка входа"""
    app = FingerLEDApp(args)
    app.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Finger count -> Arduino (OOP version)")
    parser.add_argument("--port", type=str, default="COM3",
                       help="Serial port (COMx или /dev/ttyACMx)")
    parser.add_argument("--baudrate", type=int, default=9600,
                       help="Baud rate для Arduino")
    parser.add_argument("--camera", type=int, default=0,
                       help="Индекс камеры (0 по умолчанию)")
    parser.add_argument("--simulate", action="store_true",
                       help="Не открывать serial, печатать в консоль")
    parser.add_argument("--smoothing", type=int, default=6,
                       help="Длина окна для majority-сглаживания (6 по умолчанию)")
    parser.add_argument("--resize-width", type=int, default=640,
                       help="Изменить ширину кадра для ускорения (0 - отключить)")
    
    args = parser.parse_args()
    if args.resize_width == 0:
        args.resize_width = None
    
    main(args)
