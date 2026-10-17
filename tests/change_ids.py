
from scservo_sdk import sms_sts, PortHandler

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00"
BAUDRATE = 1_000_000

CURRENT_ID = 1
NEW_ID = 3  # Change this to 3 for the third motor

port = PortHandler(PORT)
servo = sms_sts(port)

if not 0 <= NEW_ID <= 253 or NEW_ID == CURRENT_ID:
    raise ValueError("Choose a valid, different servo ID")

if not port.openPort():
    raise RuntimeError("Could not open serial port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Could not set baud rate")

    print("Checking current ID...")
    print("Ping:", servo.ping(CURRENT_ID))

    print("Unlocking EEPROM...")
    print("Unlock:", servo.unLockEprom(CURRENT_ID))

    print(f"Changing ID {CURRENT_ID} -> {NEW_ID}...")
    result = servo.write1ByteTxRx(CURRENT_ID, 5, NEW_ID)
    print("Write result:", result)

    # The servo may stop responding at its old ID immediately.
    print("Locking EEPROM at the new ID...")
    print("Lock:", servo.LockEprom(NEW_ID))

    print("Checking new ID...")
    print("New ID ping:", servo.ping(NEW_ID))

finally:
    port.closePort()
