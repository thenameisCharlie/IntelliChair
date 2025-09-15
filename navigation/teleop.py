#!/usr/bin/env python3
"""
Keyboard teleop for Yahboom G1 (SSH-friendly).
W/A/S/D = drive, SPACE = stop, +/- = speed, Q = quit.

Run:
  sudo -E python3 -u navigation/teleop.py
"""

import sys
import time
import termios
import tty
import contextlib

from hardware.motors import YahboomMotors

# Default speeds (percentage 0..100)
FWD_SPEED = 40
TURN_SPEED = 40
SPEED_STEP = 5
MIN_SPEED = 20
MAX_SPEED = 80

@contextlib.contextmanager
def raw_terminal():
    """Put stdin into raw mode so we can read single keys without Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def read_key(timeout=0.05):
    """Non-blocking single-char read; returns '' if no key pressed."""
    import select
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        ch = sys.stdin.read(1)
        # handle arrow keys (map to WASD)
        if ch == '\x1b':
            if select.select([sys.stdin], [], [], 0.001)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == '[' and select.select([sys.stdin], [], [], 0.001)[0]:
                    ch3 = sys.stdin.read(1)
                    return {'A':'w','B':'s','C':'d','D':'a'}.get(ch3, '')
            return ''
        return ch.lower()
    return ''

def try_turn(motors, side, pct):
    """Turn helper that adapts to available motor methods."""
    try:
        if side == 'left' and hasattr(motors, 'spin_left'):
            motors.spin_left(pct); return
        if side == 'right' and hasattr(motors, 'spin_right'):
            motors.spin_right(pct); return
        if side == 'left' and hasattr(motors, 'left'):
            motors.left(pct, pct); return
        if side == 'right' and hasattr(motors, 'right'):
            motors.right(pct, pct); return
        # Approximate by differential drive if available
        if side == 'left' and hasattr(motors, 'drive'):
            motors.drive(pct*0.4, pct)
        elif side == 'right' and hasattr(motors, 'drive'):
            motors.drive(pct, pct*0.4)
    except Exception:
        motors.stop()

def main():
    motors = YahboomMotors()
    speed = FWD_SPEED
    turn = TURN_SPEED

    print("\n=== TELEOP ACTIVE ===")
    print("W/A/S/D to drive, SPACE to stop, +/- to change speed, Q to quit.")
    print(f"Start speed: forward={speed}%, turn={turn}%")

    try:
        with raw_terminal():
            while True:
                key = read_key(timeout=0.05)
                if not key:
                    time.sleep(0.02)
                    continue

                if key == 'q':
                    print("\n[teleop] Quit requested.")
                    break

                if key == ' ':
                    motors.stop()
                    print("[teleop] STOP")
                    continue

                if key in ['+', '=']:
                    speed = min(MAX_SPEED, speed + SPEED_STEP)
                    turn  = min(MAX_SPEED, turn + SPEED_STEP)
                    print(f"[teleop] Speed up → fwd={speed}%, turn={turn}%")
                    continue

                if key in ['-', '_']:
                    speed = max(MIN_SPEED, speed - SPEED_STEP)
                    turn  = max(MIN_SPEED, turn - SPEED_STEP)
                    print(f"[teleop] Slow down → fwd={speed}%, turn={turn}%")
                    continue

                if key == 'w':
                    motors.forward(speed)
                    continue

                if key == 's':
                    motors.backward(speed)
                    continue

                if key == 'a':
                    try_turn(motors, 'left', turn)
                    continue

                if key == 'd':
                    try_turn(motors, 'right', turn)
                    continue

    finally:
        try:
            motors.shutdown()
        except Exception:
            pass
        print("[teleop] Clean exit.")

if __name__ == "__main__":
    main()
