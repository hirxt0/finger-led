# Finger Count → Arduino LEDs

A Python + Arduino project that counts fingers via webcam and turns on corresponding LEDs on Arduino.

## 🛠️ Requirements

### Hardware
- Arduino board (Uno, Nano, Mega, etc.)
- LEDs (5 pieces recommended)
- Resistors (220-330 Ohm for each LED)
- USB cable
- Breadboard and wires

### Software
- Python 3.x
- Arduino IDE

### Python Libraries
```bash
pip install opencv-python mediapipe pyserial
```

## 📥 Installation

1. Clone the repository:
```bash
git clone https://github.com/hirxt0/finger-led.git
cd finger-led
```

2. Install required Python packages:
```bash
pip install opencv-python mediapipe pyserial
```

3. Upload Arduino sketch:
   - Open `sketch_sep16a.ino` in Arduino IDE
   - Connect your Arduino board
   - Select correct board and port in Arduino IDE
   - Upload the sketch

## 🔌 Wiring

Connect LEDs to Arduino pins:
- LED 1 → Pin 2
- LED 2 → Pin 3
- LED 3 → Pin 4
- LED 4 → Pin 5
- LED 5 → Pin 6

Don't forget resistors (220-330 Ohm) for each LED!

## 🚀 Usage

1. Connect Arduino to your computer via USB
2. Run the Python script:
```bash
python Shhh.py
```
3. A webcam window will open
4. Show your fingers to the camera:
   - 1 finger = 1 LED on
   - 2 fingers = 2 LEDs on
   - 3 fingers = 3 LEDs on
   - 4 fingers = 4 LEDs on
   - 5 fingers = 5 LEDs on

5. Press `q` to quit

## 📝 How It Works

1. **Python script** uses MediaPipe to detect hands and count fingers
2. Sends the finger count to Arduino via Serial
3. **Arduino** receives the count and turns on corresponding number of LEDs

## ⚙️ Configuration

### Change COM Port
If you get a connection error, modify the port in `Shhh.py`:
```python
ser = serial.Serial('COM3', 9600)  # Change COM3 to your port
```

To find your port:
- **Windows**: Device Manager → Ports (COM & LPT)
- **Linux/Mac**: `/dev/ttyUSB0` or `/dev/ttyACM0`

### Adjust LED Pins
Modify pin numbers in `sketch_sep16a.ino` if needed:
```cpp
const int ledPins[] = {2, 3, 4, 5, 6};
```

## 🐛 Troubleshooting

**Problem**: "Could not open port"
- Check if Arduino is connected
- Verify correct COM port
- Close Arduino IDE Serial Monitor if open

**Problem**: LEDs don't light up
- Check wiring and resistors
- Verify Arduino sketch is uploaded
- Check Serial Monitor (9600 baud) for incoming data

**Problem**: Finger detection not working
- Ensure good lighting
- Keep hand at appropriate distance from camera
- Make sure fingers are clearly visible

## 📄 License

Free to use and modify!

## 🤝 Contributing

Feel free to open issues or submit pull requests!
