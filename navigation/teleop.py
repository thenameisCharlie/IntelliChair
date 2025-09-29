#!/usr/bin/env python3
import sys, time, termios, tty, contextlib, argparse, select
from hardware.motors import YahboomMotors

FWD_SPEED_DEFAULT  = 40
TURN_SPEED_DEFAULT = 65     # ↑ stronger turn by default
MIN_SPEED_DEFAULT  = 30
MAX_SPEED_DEFAULT  = 85
SPEED_STEP         = 5
TICK_DT            = 0.05   # 50 ms control tick

@contextlib.contextmanager
def raw_terminal():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd); yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def read_key(timeout=0.0):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if not dr: return ''
    ch = sys.stdin.read(1)
    if ch == '\x03':  # Ctrl-C
        raise KeyboardInterrupt
    if ch == '\x1b': # arrows → WASD
        if select.select([sys.stdin], [], [], 0.001)[0] and sys.stdin.read(1) == '[':
            m = {'A':'w','B':'s','C':'d','D':'a'}
            if select.select([sys.stdin], [], [], 0.001)[0]:
                return m.get(sys.stdin.read(1), '')
        return ''
    return ch.lower()

def clamp(v, lo, hi): return max(lo, min(hi, v))

def command_motors(motors, cmd, fwd_pct, turn_pct):
    """Continuously called each tick (keepalive)."""
    if cmd == 'forward':
        motors.forward(fwd_pct)
    elif cmd == 'backward':
        motors.backward(fwd_pct)
    elif cmd == 'left':
        # Prefer true in-place spin if available
        if hasattr(motors, 'spin_left'):
            motors.spin_left(turn_pct)
        # Fallback: differential drive, left tread slower/backward, right forward
        elif hasattr(motors, 'drive'):
            # In-place: left backward, right forward (strong turn)
            motors.drive(-turn_pct, turn_pct)
        elif hasattr(motors, 'left'):
            motors.left(turn_pct, turn_pct)
        else:
            # last resort: small nudge
            motors.backward(int(turn_pct*0.4)); time.sleep(0.05); motors.stop()
    elif cmd == 'right':
        if hasattr(motors, 'spin_right'):
            motors.spin_right(turn_pct)
        elif hasattr(motors, 'drive'):
            motors.drive(turn_pct, -turn_pct)
        elif hasattr(motors, 'right'):
            motors.right(turn_pct, turn_pct)
        else:
            motors.backward(int(turn_pct*0.4)); time.sleep(0.05); motors.stop()
    else:
        motors.stop()

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fwd",  type=int, default=FWD_SPEED_DEFAULT)
    ap.add_argument("--turn", type=int, default=TURN_SPEED_DEFAULT)
    ap.add_argument("--min",  dest="min_speed", type=int, default=MIN_SPEED_DEFAULT)
    ap.add_argument("--max",  dest="max_speed", type=int, default=MAX_SPEED_DEFAULT)
    return ap.parse_args()

def main():
    args   = parse_args()
    speed  = clamp(args.fwd,  args.min_speed, args.max_speed)
    turn   = clamp(args.turn, args.min_speed, args.max_speed)
    motors = YahboomMotors()

    print("\n=== TELEOP === W/A/S/D move | SPACE stop | +/- speed | Q quit")
    print(f"fwd={speed}% turn={turn}%")

    current_cmd = 'stop'
    try:
        with raw_terminal():
            last_print = 0.0
            while True:
                # 1) read any pending key quickly (non-blocking)
                key = read_key(timeout=0.0)

                if key == 'q': break
                if key == ' ':
                    current_cmd = 'stop'
                    motors.stop()
                elif key in ['+', '=']:
                    speed = clamp(speed + SPEED_STEP, args.min_speed, args.max_speed)
                    turn  = clamp(turn  + SPEED_STEP, args.min_speed, args.max_speed)
                elif key in ['-', '_']:
                    speed = clamp(speed - SPEED_STEP, args.min_speed, args.max_speed)
                    turn  = clamp(turn  - SPEED_STEP, args.min_speed, args.max_speed)
                elif key == 'w':
                    current_cmd = 'forward'
                elif key == 's':
                    current_cmd = 'backward'
                elif key == 'a':
                    current_cmd = 'left'
                elif key == 'd':
                    current_cmd = 'right'
                # else: no new key → keep last command

                # 2) keepalive: re-send the command every tick
                command_motors(motors, current_cmd, speed, turn)

                # small status print (throttled)
                now = time.time()
                if now - last_print > 0.5:
                    print(f"\r[cmd={current_cmd:8s}] fwd={speed:2d}% turn={turn:2d}%", end="", flush=True)
                    last_print = now

                time.sleep(TICK_DT)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            motors.stop()
            motors.shutdown()
        except Exception:
            pass
        print("\n[teleop] exit")

if __name__ == "__main__":
    main()
