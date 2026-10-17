
import time
from scservo_sdk import sms_sts, PortHandler

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00"
BAUDRATE = 1_000_000
SERVO_ID = 3

port = PortHandler(PORT)
servo = sms_sts(port)

if not port.openPort():
    raise RuntimeError("Could not open serial port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Could not set baud rate")

    # Enable motor torque (register 40)
    print("Torque enable:",
          servo.write1ByteTxRx(SERVO_ID, 40, 1))

    # Set continuous-rotation mode
    print("Wheel mode:", servo.WheelMode(SERVO_ID))

    # Give the servo a moment to apply the settings
    time.sleep(0.2)

    # Command rotation at a moderate test speed
    print("Speed command:", servo.WriteSpec(SERVO_ID, 500, 10))

    for _ in range(1000):
        time.sleep(1)
        print("Position, speed, communication, error:",
              servo.ReadPosSpeed(SERVO_ID))

    # Stop
    print("Stop command:", servo.WriteSpec(SERVO_ID, 0, 20))

finally:
    try:
        servo.WriteSpec(SERVO_ID, 0, 20)
    except Exception:
        pass
    port.closePort()
