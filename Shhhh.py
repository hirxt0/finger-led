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
    SerialException = Exceptoin

def fingers_up_from_landmarks(landmarks, handedness_label):
    TIP_IDS = [4, 8, 12, 16, 20]
    PIP_IDS = [3, 6, 10, 14, 18]
    fingers = [False] * 5
    if handedness_label == 'Right':
        fingers[0] = landmarks[TIP_IDS[0]].x < landmarks[PIP_IDS[0]].x
    else:    
        fingers[0] = landmarks[TIP_IDS[0]].x > landmarks[PIP_IDS[0]].x
    
    for i in range(1, 5):
        tip_id = TIP_IDS[i]
        pip_id = PIP_IDS[i]
        fingers[i] = landmarks[tip_id].y < landmarks[pip_id].y
    return fingers

def main(args):
    ser = None
    if not args.simulate:
        if serial is None:
            raise SystemExit()
        try:
            ser = serial.Serial(args.port, args.baudrate, timeout=1)
            time.sleep(2.0)
            print(f"[Serial] Открыт порт {args.port}, {args.baudrate}")
        except SerialException as e:
            raise SystemExit()
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode = False,
        max_num_hands = 1,
        model_complexity = 1,
        min_detection_confidence = 0.6,
        min_tracking_confidence = 0.5   
    )
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit("не удалось открыть камеру")
    print("Нажмите ESC чтобы выйти")
    last_sent = None
    recent = deque(maxlen=args.smoothing)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("кадры не читаются")
                break
            if args.resize_width:
                h, w = frame.shape[:2]
                scale = args.resize_width / float(w)
                frame = cv2.resize(frame, (args.resize_width, int(h * scale)))

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            result = hands.process(rgb)

            finger_count = 0
            finger_states = None
            if result.multi_hand_landmarks:
                hand_landmarks = result.multi_hand_landmarks[0]
                handendness_label = result.multi_handedness[0].classification[0].label
                landmarks = hand_landmarks.landmark
                finger_states = fingers_up_from_landmarks(landmarks, handendness_label)
                finger_count = sum(1 for f in finger_states if f)
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                cv2.putText(frame, f"{handendness_label}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
            recent.append(finger_count)
            smoothed = Counter(recent).most_common(1)[0][0]

            if smoothed != last_sent:
                last_sent = smoothed
                if ser:
                    msg = f"{smoothed}\n"
                    try:
                        ser.write(msg.encode('utf-8'))
                    except SerialException as e:
                        print("ошибка при записи", e)
                        ser = None
                else:
                    print(f"[SIM] -> {smoothed}")
            cv2.putText(frame, f"Fingers: {finger_count} (smoothed: {smoothed})", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            if finger_states is not None:
                txt = "".join(['1' if x else '0' for x in finger_states])
                cv2.putText(frame, txt, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow("Finger -> Arduino", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        if ser:
            ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Finger count -> Arduino")
    parser.add_argument("--port", type=str, default="COM3", help="Serial port (COMx или /dev/ttyACMx)")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baud rate для Arduino")
    parser.add_argument("--camera", type=int, default=0, help="Индекс камеры (0 по умолчанию)")
    parser.add_argument("--simulate", action="store_true", help="Не открывать serial, печатать в консоль")
    parser.add_argument("--smoothing", type=int, default=6, help="Длина окна для majority-сглаживания (6 по умолчанию)")
    parser.add_argument("--resize-width", type=int, default=640, help="Изменить ширину кадра для ускорения (0 - отключить)")
    args = parser.parse_args()
    if args.resize_width == 0:
        args.resize_width = None
    main(args)