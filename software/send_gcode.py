"""
send_gcode.py

Streams a .gcode file to the Arduino one line at a time, waiting
for "ok" after each line before sending the next. This matches
the handshake xy_gcode_plotter*.ino uses (Serial.println("ok")
after every processed line).

Long moves (e.g. the first rapid jump from the home corner to the
start of a shape) can legitimately take several seconds to finish
on the Arduino side -- it only prints "ok" once the move is done.
Rather than using a single short pyserial read-timeout (which gives
up mid-move and looks like a dead connection), this script polls in
small increments up to a much longer overall per-line timeout, and
prints a periodic heartbeat so you can see it's still alive.

Usage:
    python send_gcode.py circle.gcode COM3
    python send_gcode.py circle.gcode COM3 9600
    python send_gcode.py circle.gcode COM3 9600 60   (60s per-line timeout)

    (On Linux/Mac the port looks like /dev/ttyUSB0 or /dev/ttyACM0
    instead of COM3 -- check with `ls /dev/tty*` while plugged in.)
"""

import serial
import sys
import time

POLL_INTERVAL = 1.0        # seconds per readline() attempt while waiting
HEARTBEAT_EVERY = 3.0      # print "...still waiting" at least this often


def wait_for_ok(ser, line_desc, max_wait):
    """Poll for a response line, tolerating long-running moves.

    Returns True once "ok" is seen. Prints any other non-empty lines
    the Arduino sends along the way (debug/status messages). Returns
    False only if truly nothing arrives within max_wait seconds.
    """
    waited = 0.0
    last_heartbeat = 0.0

    while waited < max_wait:
        response = ser.readline().decode(errors="replace").strip()

        if response:
            if "ok" in response.lower():
                return True
            print("<<", response)
            waited = 0.0  # got *something* -- reset the stall clock
            last_heartbeat = 0.0
            continue

        waited += POLL_INTERVAL
        last_heartbeat += POLL_INTERVAL
        if last_heartbeat >= HEARTBEAT_EVERY:
            print(f"   ... still waiting for '{line_desc}' ({waited:.0f}s/{max_wait:.0f}s)")
            last_heartbeat = 0.0

    return False


def send_gcode(filepath, port, baud=9600, max_wait=60):
    ser = serial.Serial(port, baud, timeout=POLL_INTERVAL)
    time.sleep(2)  # let the Arduino reset after the port opens

    # Drain any startup messages ("Homing...", "Ready", first "ok")
    time.sleep(2)
    while ser.in_waiting:
        print("<<", ser.readline().decode(errors="replace").strip())

    with open(filepath) as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    total = len(lines)
    for i, line in enumerate(lines, 1):
        ser.write((line + "\n").encode())
        print(f">> [{i}/{total}] {line}")

        if not wait_for_ok(ser, line, max_wait):
            print(f"!! No response for line {i} ('{line}') after {max_wait}s -- check connection")
            break

    print("Done streaming.")
    ser.close()


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python send_gcode.py <file.gcode> <port> [baud] [max_wait_seconds]")
        sys.exit(1)

    gcode_file = sys.argv[1]
    port = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 9600
    max_wait = float(sys.argv[4]) if len(sys.argv) > 4 else 60

    send_gcode(gcode_file, port, baud, max_wait)
