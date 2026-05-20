import argparse
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description="Send TX Wi-Fi power to the ESP32 over serial.")
    parser.add_argument("port", help="Serial port, for example COM22")
    parser.add_argument("power", type=int, help="TX power value from 8 to 80")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--delay", type=float, default=1.0, help="Delay after opening the port, in seconds")
    args = parser.parse_args()

    if args.power < 8 or args.power > 80:
        print("Power must be between 8 and 80.", file=sys.stderr)
        return 2

    print(f"Opening {args.port} at {args.baud}...")
    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.5,
            write_timeout=5,
            rtscts=False,
            dsrdtr=False,
        ) as ser:
            time.sleep(args.delay)

            line = f"{args.power}\n".encode("ascii")
            print(f"Sending: {args.power}")
            ser.reset_output_buffer()
            ser.write(line)
            ser.flush()

            deadline = time.time() + 2
            while time.time() < deadline:
                response = ser.readline().decode("utf-8", errors="replace").strip()
                if response:
                    print(response)
                    if "TX_POWER=" in response:
                        return 0

            print("Sent, but no TX_POWER confirmation received before timeout.")
            return 1
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        print("Close miniterm / idf.py monitor / Arduino Serial Monitor if one is open.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
