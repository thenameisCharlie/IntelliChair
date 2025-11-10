#!/usr/bin/env python3
"""
Intellichair unified entrypoint.

Examples:
  python3 main.py teleop
  python3 main.py autonomy
  python3 main.py teach
  python3 main.py teach-auto
  python3 main.py test-ultrasonic --samples 10
  python3 main.py test-motors
  python3 main.py test-leds
  python3 main.py servo-center



==============================
 Intellichair Main.py Modes
==============================

1. teleop
   Command:
     python3 main.py teleop
   Description:
     - Manual driving mode using the keyboard.
     - Controls: W/A/S/D to move, SPACE to stop, Q to quit.
     - Useful for testing motors or manually exploring a space.

2. autonomy
   Command:
     python3 main.py autonomy
   Description:
     - Fully automatic navigation mode.
     - Robot drives forward, slows near obstacles, and stops/turns when blocked.
     - Uses the ultrasonic sensor + servo sweep to pick a free direction.
     - SLAM runs in the background; map is saved on exit.

3. teach
   Command:
     python3 main.py teach
   Description:
     - Manual mapping mode.
     - Starts SLAM while you drive the robot manually (teleop).
     - When you quit teleop, it saves the generated map.
     - Ideal for “teaching” the robot a room layout.

4. teach-auto
   Command:
     python3 main.py teach-auto
   Description:
     - Hybrid mapping + autonomy.
     - First runs teleop with SLAM so you can manually explore and build a map.
     - When you quit teleop, the system automatically saves the map and switches into autonomy mode.
     - Great for demos: “We map manually, then it drives itself.”
"""

import sys, time, argparse

# ---------- Imports with graceful fallbacks ----------
def _load_modules():
    errs = []

    # config
    try:
        from utils.config import TUNABLES
    except Exception as e:
        print(f"[config] Could not import utils.config.TUNABLES: {e}")
        sys.exit(1)

    # hardware
    HW = {}
    try:
        from hardware.motors import YahboomMotors
        HW["motors_cls"] = YahboomMotors
    except Exception as e:
        errs.append(f"hardware.motors: {e}")
    try:
        from hardware.ultrasonic import distance_filtered_cm
        HW["ultra_fn"] = distance_filtered_cm
    except Exception as e:
        errs.append(f"hardware.ultrasonic: {e}")
    try:
        from hardware.servo import set_angle, center
        HW["servo_set"] = set_angle
        HW["servo_center"] = center
    except Exception as e:
        errs.append(f"hardware.servo: {e}")
    try:
        from hardware.leds import red, green, blue, yellow, magenta, cyan, off
        HW["leds"] = (red, green, blue, yellow, magenta, cyan, off)
    except Exception:
        HW["leds"] = (lambda:None,)*7  # optional LEDs

    # perception
    PC = {}
    try:
        from perception.slam import start_slam, save_map
        PC["start_slam"] = start_slam
        PC["save_map"]  = save_map
    except Exception as e:
        errs.append(f"perception.slam: {e}")

    # lidar (optional; may require matplotlib)
    try:
        from perception.lidar import get_scan
        PC["get_scan"] = get_scan
    except Exception:
        pass

    # navigation (teleop defines main(), not run())
    NV = {}
    try:
        from navigation.teleop import main as teleop_main
        NV["teleop_main"] = teleop_main
    except Exception:
        pass

    return TUNABLES, HW, PC, NV, errs

# Load modules first so TUNABLES exists before we sanitize values.
TUNABLES, HW, PC, NV, _IMPORT_ERRS = _load_modules()

# ---------- Tunables & fallback defaults ----------
def num(x, default):
    """Coerce config values into a float: supports int/float/str and [0]/(0,)."""
    try:
        if isinstance(x, (int, float)):
            return float(x)
        if isinstance(x, (list, tuple)) and len(x) > 0:
            return float(x[0])
        if isinstance(x, str):
            return float(x.strip())
    except Exception:
        pass
    return float(default)

THRESH_CM    = num(TUNABLES.get("THRESH_CM", 30), 30)
CRUISE_SPEED = num(TUNABLES.get("CRUISE_SPEED", 40), 40)
SLOW_SPEED   = num(TUNABLES.get("SLOW_SPEED", 25), 25)
SLOW_BAND    = num(TUNABLES.get("SLOW_BAND", 10), 10)
SPIN_SPEED   = num(TUNABLES.get("SPIN_SPEED", 50), 50)
SPIN_TIME_S  = num(TUNABLES.get("SPIN_TIME_S", 0.6), 0.6)
LOOP_DT      = num(TUNABLES.get("LOOP_DT", 0.05), 0.05)

STOP_CM      = THRESH_CM
RESUME_CM    = STOP_CM + 8.0

# --- add these helpers once ---
def _turn_left(m, speed):
    if hasattr(m, "turn_left"):
        m.turn_left(speed)
    elif hasattr(m, "spin_left"):
        m.spin_left(speed)
    else:
        raise AttributeError("Motor driver has neither turn_left nor spin_left")

def _turn_right(m, speed):
    if hasattr(m, "turn_right"):
        m.turn_right(speed)
    elif hasattr(m, "spin_right"):
        m.spin_right(speed)
    else:
        raise AttributeError("Motor driver has neither turn_right nor spin_right")


# ---------- LED helpers (never crash even if GPIO mode was cleaned up) ----------
def leds_safe():
    if "leds" not in HW:
        return (lambda:None,)*7
    funcs = HW["leds"]
    def wrap(f):
        def _inner():
            try:
                f()
            except Exception:
                # GPIO may have been cleaned; ignore LED errors
                pass
        return _inner
    return tuple(wrap(f) for f in funcs)

# ---------- Helpers ----------
def pick_turn_direction(sample_fn, servo_set_angle, servo_center):
    """Sweep servo right/center/left, read distances, pick the best side."""
    readings = []
    for angle, label in [(45, "right"), (90, "center"), (135, "left")]:
        servo_set_angle(angle)
        time.sleep(0.15)
        try:
            d = float(sample_fn())
        except Exception:
            d = 0.0
        readings.append((d, label))
    servo_center()
    best_d, best_label = max(readings, key=lambda x: x[0])
    if best_label == "center":
        sides = [(d, lbl) for d, lbl in readings if lbl != "center"]
        return max(sides, key=lambda x: x[0])[1]
    return best_label

def _delegate_to_teleop():
    """Call navigation.teleop.main() without leaving the subcommand in argv."""
    if "teleop_main" not in NV:
        print("[teleop] navigation.teleop.main() not found.")
        return 1
    # sys.argv = [prog, 'teleop', ...] -> [prog, ...]
    sys.argv = [sys.argv[0]] + sys.argv[2:]
    return NV["teleop_main"]()

# ---------- Modes ----------
def mode_autonomy():
    """
    Robust obstacle avoidance with recovery:
    - Emergency brake
    - Median-of-5 sampling + periodic microscan
    - Back-off after stop, then turn toward freer side, then nudge forward
    - Escalating escape maneuver if repeatedly blocked
    """
    missing = [k for k in ("motors_cls","ultra_fn","servo_set","servo_center") if k not in HW]
    if missing:
        print(f"[autonomy] Missing modules: {missing}. Ensure motors/ultrasonic/servo are implemented.")
        return

    red, green, blue, yellow, magenta, cyan, off = leds_safe()
    m = HW["motors_cls"]()

    # --- quick-tune params ---
    EMERGENCY_CM          = max(10.0, STOP_CM - 5.0)  # hard stop under this
    CLEAR_HITS_REQUIRED   = 3
    MICROSCAN_EVERY       = 10
    MICROSCAN_ANGLE       = 15
    N_MEDIAN_SAMPLES      = 5
    MAX_VALID_CM          = 400.0
    MIN_VALID_CM          = 5.0

    BACKUP_SPEED          = max(15.0, SLOW_SPEED)   # reverse speed
    BACKUP_TIME_S         = 0.45                    # reverse duration
    NUDGE_TIME_S          = 0.25                    # forward nudge after turn
    ESCAPE_COUNT_THRESH   = 3                       # if we stop this many times in window, do bigger escape
    ESCAPE_WINDOW_S       = 4.0                     # time window for counting stops
    ESCAPE_TURN_TIME_S    = max(0.9, SPIN_TIME_S*1.5)  # bigger turn

    DEBUG = True

    # --- helpers ---
    import time as _time
    recent_stops = []  # timestamps of recent stops

    def sample_cm() -> float:
        """Median-of-N with sanitization; returns cm; 999 if unknown."""
        vals = []
        for _ in range(N_MEDIAN_SAMPLES):
            try:
                v = float(HW["ultra_fn"]())
            except Exception:
                v = float("nan")
            if not (MIN_VALID_CM <= v <= MAX_VALID_CM):
                v = float("nan")
            vals.append(v)
            time.sleep(0.002)
        valid = [v for v in vals if not (v != v)]
        if not valid:
            return 999.0
        valid.sort()
        return valid[len(valid)//2]

    def microscan_forward_cm() -> float:
        """Quick mini-scan: left/center/right small angles; return min (closest)."""
        try:
            HW["servo_set"](90 - MICROSCAN_ANGLE); time.sleep(0.06)
            left = sample_cm()
            HW;                    time.sleep(0.06)
            center = sample_cm()
            HW["servo_set"](90 + MICROSCAN_ANGLE); time.sleep(0.06)
            right = sample_cm()
        finally:
            HW["servo_center"]()
        return min(left, center, right)

    def record_stop_and_should_escape() -> bool:
        """Track recent stops; return True if we should escalate to a bigger escape."""
        now = _time.time()
        recent_stops.append(now)
        # keep only recent
        while recent_stops and (now - recent_stops[0]) > ESCAPE_WINDOW_S:
            recent_stops.pop(0)
        return len(recent_stops) >= ESCAPE_COUNT_THRESH

    def back_off_then_turn(turn_dir: str, big: bool = False):
        """Reverse, turn (bigger if 'big'), then nudge forward."""
        # back off
        if DEBUG: print(f"[auto] BACK OFF {BACKUP_TIME_S:.2f}s")
        m.backward(BACKUP_SPEED); time.sleep(BACKUP_TIME_S)
        m.stop()

        # turn
        ttime = ESCAPE_TURN_TIME_S if big else SPIN_TIME_S
        if DEBUG: print(f"[auto] TURN {turn_dir.upper()} for {ttime:.2f}s")
        if turn_dir == "left":
            _turn_left(m, SPIN_SPEED); time.sleep(ttime)
        else:
            _turn_right(m, SPIN_SPEED); time.sleep(ttime)
        m.stop()

        # nudge forward to clear the corner/sensor cone
        if DEBUG: print(f"[auto] NUDGE {NUDGE_TIME_S:.2f}s")
        m.forward(SLOW_SPEED); time.sleep(NUDGE_TIME_S)
        m.stop()

    HW["servo_center"]()
    green()

    slam_handle = None
    if "start_slam" in PC:
        try:
            print("[autonomy] starting SLAM…")
            slam_handle = PC["start_slam"]()
        except Exception as e:
            print(f"[autonomy] start_slam error: {e}")

    clear_hits = 0
    tick = 0

    try:
        while True:
            d = sample_cm()
            tick += 1

            # periodic microscan
            if tick % MICROSCAN_EVERY == 0:
                d = min(d, microscan_forward_cm())

            # EMERGENCY BRAKE
            if d < EMERGENCY_CM:
                m.stop(); red()
                if DEBUG: print(f"[auto] EMERGENCY STOP (d={d:.1f}cm < {EMERGENCY_CM}cm)")
                should_escape = record_stop_and_should_escape()
                # pick best direction and recover
                turn = pick_turn_direction(sample_cm, HW["servo_set"], HW["servo_center"])
                back_off_then_turn(turn, big=should_escape)
                clear_hits = 0
                time.sleep(0.05)
                continue

            # state report
            if DEBUG:
                state = 'BLOCK' if d <= STOP_CM else ('SLOW' if d <= STOP_CM + SLOW_BAND else 'CLEAR')
                print(f"[auto] d={d:.1f}cm  stop<={STOP_CM:.1f}  slow<={STOP_CM+SLOW_BAND:.1f}  "
                      f"state={state}  clears={clear_hits}")

            # main logic
            if d <= STOP_CM:
                # BLOCKED: stop → decide → back off → turn → nudge
                m.stop(); red()
                should_escape = record_stop_and_should_escape()
                turn = pick_turn_direction(sample_cm, HW["servo_set"], HW["servo_center"])
                if DEBUG: print(f"[auto] TURN {turn.upper()} (blocked)")
                back_off_then_turn(turn, big=should_escape)
                clear_hits = 0
                yellow()

            elif d <= STOP_CM + SLOW_BAND:
                clear_hits = 0
                if DEBUG: print("[auto] SLOW")
                m.forward(SLOW_SPEED); cyan()

            else:
                clear_hits += 1
                if clear_hits >= CLEAR_HITS_REQUIRED:
                    if DEBUG: print("[auto] CRUISE")
                    m.forward(CRUISE_SPEED); green()
                else:
                    if DEBUG: print("[auto] VERIFY CLEAR → SLOW UNTIL CONFIRMED")
                    m.forward(SLOW_SPEED); cyan()

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\n[autonomy] stopping…")
    finally:
        try: m.stop()
        except Exception: pass
        blue()
        if slam_handle and "save_map" in PC:
            try:
                PC["save_map"](slam_handle)
                print("[autonomy] map saved.")
            except Exception as e:
                print(f"[autonomy] save_map error: {e}")
        off()

def mode_teleop():
    rc = _delegate_to_teleop()
    if rc is None: rc = 0
    return rc

def mode_teach(switch_to_auto: bool):
    """
    Start SLAM, then run teleop so you can drive and map manually.
    On exit, save the map; if switch_to_auto=True, chain into autonomy.
    """
    red, green, blue, yellow, magenta, cyan, off = leds_safe()
    slam_handle = None

    if "start_slam" in PC:
        try:
            print("[teach] starting SLAM (manual mapping)…")
            slam_handle = PC["start_slam"]()
            green()
        except Exception as e:
            print(f"[teach] start_slam error: {e}")

    try:
        rc = _delegate_to_teleop()
        if rc not in (None, 0):
            print(f"[teach] teleop exited with code {rc}")
    finally:
        if slam_handle and "save_map" in PC:
            try:
                blue()
                PC["save_map"](slam_handle)
                print("[teach] map saved.")
            except Exception as e:
                print(f"[teach] save_map error: {e}")
        # Do not let LEDs crash if GPIO got cleaned by other modules
        try:
            off()
        except Exception:
            pass

    if switch_to_auto:
        print("[teach] switching to autonomy…")
        mode_autonomy()

def mode_test_ultrasonic(samples: int):
    if "ultra_fn" not in HW:
        print("[test-ultrasonic] ultrasonic not available.")
        return
    print("[test-ultrasonic] readings (cm):")
    for _ in range(samples):
        try:
            d = float(HW["ultra_fn"]())
            print(f"{d:.1f}")
        except Exception as e:
            print(f" read error: {e}")
        time.sleep(0.1)

def mode_test_motors():
    if "motors_cls" not in HW:
        print("[test-motors] motors not available.")
        return
    m = HW["motors_cls"]()
    try:
        print("[test-motors] forward/back/left/right/stop")
        m.forward(40);  time.sleep(0.8)
        m.backward(40); time.sleep(0.8)
        _turn_left(m, 50); time.sleep(0.6)
        _turn_right(m, 50); time.sleep(0.6)
        m.stop()
    finally:
        m.stop()


def mode_test_leds():
    red, green, blue, yellow, magenta, cyan, off = leds_safe()
    for f in (red, green, blue, yellow, magenta, cyan, off):
        try:
            f(); time.sleep(0.3)
        except Exception as e:
            print(f"[test-leds] LED function error: {e}")
    off()
    print("[test-leds] done.")

def mode_servo_center():
    if "servo_center" not in HW:
        print("[servo] servo.center not available.")
        return
    HW["servo_center"]()
    print("[servo] centered.")

# ---------- CLI ----------
def build_parser():
    p = argparse.ArgumentParser(description="Intellichair controller")
    sub = p.add_subparsers(dest="mode", required=True)

    sub.add_parser("autonomy", help="Run SLAM + obstacle avoidance")
    sub.add_parser("teleop", help="Run teleop controls")
    sub.add_parser("teach", help="Start SLAM, then teleop for manual mapping; save map on exit")
    sub.add_parser("teach-auto", help="teach; when you quit teleop, automatically switch to autonomy")

    t = sub.add_parser("test-ultrasonic", help="Read ultrasonic N times")
    t.add_argument("--samples", type=int, default=10)

    sub.add_parser("test-motors", help="Exercise motors")
    sub.add_parser("test-leds", help="Cycle RGB LEDs")
    sub.add_parser("servo-center", help="Center the servo")

    #will allow for one liner when saving location on terminal
    sp = sub.add_parser("save-place", help="Save current pose under a name")
    sp.add_argument("--name", required=True, help="Place name (e.g., kitchen)")
    sp.add_argument("--aliases", default="", help="Optional aliases, comma-separated")

    sub.add_parser("list-places", help="List saved place names")

    return p

def main():
    args = build_parser().parse_args()
    if args.mode == "autonomy":          mode_autonomy()
    elif args.mode == "teleop":          mode_teleop()
    elif args.mode == "teach":           mode_teach(switch_to_auto=False)
    elif args.mode == "teach-auto":      mode_teach(switch_to_auto=True)
    elif args.mode == "test-ultrasonic": mode_test_ultrasonic(args.samples)
    elif args.mode == "test-motors":     mode_test_motors()
    elif args.mode == "test-leds":       mode_test_leds()
    elif args.mode == "servo-center":    mode_servo_center()

    elif args.mode == "save-place":
        # get current pose (works with your perception.lidar/get_pose)
        try:
            from perception.lidar import get_pose
            pose = get_pose()  # dict or (x,y,theta) depending on your impl
        except Exception as e:
            print(f"[save-place] Could not read pose: {e}")
            return 1

        from navigation.places import PlaceManager, Pose
        pm = PlaceManager()
        # normalize pose to Pose dataclass
        if isinstance(pose, dict):
            x, y, th = float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("theta", 0.0))
        else:
            x, y, th = float(pose[0]), float(pose[1]), float(pose[2])

        name = args.name.strip().lower()
        aliases = [a.strip().lower() for a in args.aliases.split(",") if a.strip()]
        pm.add_place(name, Pose(x, y, th), aliases=aliases)
        pm._save()
        print(f"Saved '{name}' at (x={x:.3f}, y={y:.3f}, θ={th:.3f}) aliases={aliases}")

    elif args.mode == "list-places":
        from navigation.places import PlaceManager
        pm = PlaceManager()
        names = pm.list_places()
        print("Places:", ", ".join(names) if names else "(none)")


if __name__ == "__main__":
    main()
