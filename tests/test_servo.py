from scservo_sdk import sms_sts, PortHandler

# Replace this with the port reported by your Pi.
PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
SERVO_ID = 1

port = PortHandler(PORT)
servo = sms_sts(port)

if not port.openPort():
    raise RuntimeError(f"Could not open {PORT}")

if not port.setBaudRate(BAUDRATE):
    port.closePort()
    raise RuntimeError(f"Could not set baud rate to {BAUDRATE}")

try:
    result = servo.ping(SERVO_ID)
    print(f"Ping result for servo {SERVO_ID}: {result}")
finally:
    port.closePort()
