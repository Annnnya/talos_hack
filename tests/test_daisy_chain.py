
#!/usr/bin/env python3
import time
from scservo_sdk import sms_sts, PortHandler

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00"
BAUDRATE = 1_000_000

SERVO_IDS = [1, 2, 3]
SPEED = 300       # Try a low speed first
ACCELERATION = 20
RUN_SECONDS = 60

port = PortHandler(PORT)
servo = sms_sts(port)


def check_result(result, operation):
    # SDK write methods typically return (communication_result, error)
    print(f"{operation}: {result}")
    if result[0] != 0 or result[1] != 0:
        raise RuntimeError(f"{operation} failed: {result}")


def stop_all():
    for servo_id in SERVO_IDS:
        try:
            servo.WriteSpec(servo_id, 0, ACCELERATION)
        except Exception as exc:
            print(f"Warning: could not stop servo {servo_id}: {exc}")


if not port.openPort():
    raise RuntimeError(f"Could not open {PORT}")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError(f"Could not set baud rate {BAUDRATE}")

    # First, verify that all three IDs respond.
    for servo_id in SERVO_IDS:
        result = servo.ping(servo_id)
        print(f"Servo {servo_id} ping: {result}")

    # Configure each servo for continuous rotation.
    for servo_id in SERVO_IDS:
        check_result(
            servo.WheelMode(servo_id),
            f"Wheel mode for servo {servo_id}"
        )
        check_result(
            servo.write1ByteTxRx(servo_id, 40, 1),
            f"Torque enable for servo {servo_id}"
        )

    print(f"\nRotating all three servos at speed {SPEED}...")
    try:
        for servo_id in SERVO_IDS:
            check_result(
                servo.WriteSpec(servo_id, SPEED, ACCELERATION),
                f"Start servo {servo_id}"
            )

        time.sleep(RUN_SECONDS)

    finally:
        print("\nStopping all servos...")
        stop_all()

    print("Test finished.")

finally:
    stop_all()
    port.closePort()
