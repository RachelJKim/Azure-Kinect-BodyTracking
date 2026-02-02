#!/usr/bin/env python3
import argparse
import csv
import signal
import socket
import struct
import sys
import time

try:
    import msvcrt
except ImportError:
    msvcrt = None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Receive UDP binary packets and save them to a file."
    )
    parser.add_argument("--ip", default="192.168.0.65", help="IP to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=7000, help="UDP port (default: 7000)")
    parser.add_argument(
        "--output",
        default="udp_capture.csv",
        help="Output CSV file path (default: udp_capture.csv)",
    )
    parser.add_argument(
        "--max-packets",
        type=int,
        default=0,
        help="Stop after N packets (0 = run forever)",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=60,
        help="Print a status line every N packets (default: 60)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.settimeout(0.1)

    packet_count = 0
    byte_count = 0
    start_time = time.time()
    recording = False
    running = True

    def handle_signal(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, handle_signal)

    expected_float_count = None
    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        print(f"Listening on {args.ip}:{args.port} -> {args.output}")
        print("Press 's' to start/stop recording, 'q' or Ctrl+Z to quit.")
        while running:
            if msvcrt and msvcrt.kbhit():
                key = msvcrt.getch()
                if key in (b"s", b"S"):
                    recording = not recording
                    state = "ON" if recording else "OFF"
                    print(f"Recording: {state}")
                elif key in (b"q", b"Q", b"\x1a"):
                    running = False
                    break

            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            packet_count += 1
            byte_count += len(data)

            if not recording:
                continue

            float_count = len(data) // 4
            remainder = len(data) % 4
            if remainder != 0:
                data = data[: len(data) - remainder]

            if expected_float_count is None:
                expected_float_count = float_count
                header = ["timestamp", "source_ip", "source_port", "byte_len"]
                header += [f"f{i}" for i in range(expected_float_count)]
                writer.writerow(header)

            if float_count != expected_float_count:
                float_count = expected_float_count

            floats = struct.unpack("<" + "f" * float_count, data[: float_count * 4])
            row = [time.time(), addr[0], addr[1], len(data)]
            row.extend(floats)
            writer.writerow(row)

            if args.print_every > 0 and packet_count % args.print_every == 0:
                elapsed = max(time.time() - start_time, 0.001)
                rate = byte_count / elapsed
                print(
                    f"{packet_count} packets from {addr[0]}:{addr[1]} | "
                    f"{byte_count} bytes total | {rate:.1f} B/s"
                )

            if args.max_packets > 0 and packet_count >= args.max_packets:
                break

    print("Done.")


if __name__ == "__main__":
    main()
