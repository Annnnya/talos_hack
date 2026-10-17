
#!/usr/bin/env python3
import time
from scservo_sdk import sms_sts, PortHandler

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00"
BAUDRATE = 1_000_000
SERVO_ID = 1

port = PortHandler(PORT)
servo = sms_sts(port)

if not port.openPort():
    raise RuntimeError("Could not open serial port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Could not set baud rate")

    # Enable continuous-rotation (wheel) mode
    result = servo.WheelMode(SERVO_ID)
    print("Wheel mode result:", result)

    # Rotate slowly in one direction for 1 second
    print("Rotating...")
    result = servo.WriteSpec(SERVO_ID, 100, 20)
    print("Speed command result:", result)

    time.sleep(1)

    # Stop the motor
    servo.WriteSpec(SERVO_ID, 0, 20)
    print("Stopped")

finally:
    # Make a best effort to stop before closing
    try:
        servo.WriteSpec(SERVO_ID, 0, 20)
    except Exception:
        pass
    port.closePort()
