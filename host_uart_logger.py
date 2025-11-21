import argparse
import datetime
import sys

try:
    import serial
except ImportError:
    print("pyserial not installed. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Read debug logs from Pico UART")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM3)")
    parser.add_argument("-b", "--baud", type=int, default=1000000, help="Baud rate (default: 1000000)")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        print(f"Opened {args.port} @ {args.baud}")
        while True:
            line = ser.readline()
            if not line:
                continue
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            try:
                text = line.decode(errors="replace").rstrip()
            except Exception:
                text = repr(line)
            print(f"{ts} | {text}")


if __name__ == "__main__":
    main()
