import serial, struct, numpy as np, cv2

PORT = 'COM4'          # change if your ESP32 uses a different COM port
BAUD = 921600          # must match Serial.begin() in Arduino
MAGIC = b'IMG'

def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise RuntimeError("Serial timeout")
        buf += chunk
    return bytes(buf)

with serial.Serial(PORT, BAUD, timeout=3) as ser:
    print(f"Listening on {PORT} at {BAUD} baud...")
    while True:
        sync = ser.read_until(MAGIC)
        if not sync.endswith(MAGIC):
            continue

        raw_len = read_exact(ser, 4)
        (length,) = struct.unpack('>I', raw_len)
        if length == 0 or length > 2_000_000:
            print("Bad length:", length)
            continue

        jpg = read_exact(ser, length)

        img_array = np.frombuffer(jpg, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if img is None:
            print("Decode fail")
            continue

        cv2.imshow('ESP32 Camera', img)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

cv2.destroyAllWindows()
