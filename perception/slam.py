"""
perception/slam.py
SLAM orchestration for Intellichair.

- start_slam(): launch slam_toolbox online_async and return a Popen handle
- save_map(handle, name=None): attempt to serialize map into ./maps/<name>
  * If ROS2 serialize service is unavailable, write a harmless stub file
- load_map(name): stub loader (extend if you rely on loading)
- _get_current_pose_from_lidar(): stub (extend if you publish/subscribe TF)
- teach_place(name): stub that records a named place with current pose (if available)

This module intentionally avoids any GPIO usage to prevent crashes if
GPIO pin mode has been cleaned up elsewhere.
"""

from __future__ import annotations

import os
import time
import json
import subprocess
import shutil
from pathlib import Path
from typing import Optional, Tuple, Any

try:
    from perception.lidar import get_pose as _lidar_get_pose
except Exception:
    _lidar_get_pose = None

# Directory where maps and stubs are saved
MAPS_DIR = Path("maps")

POSE_FILE = Path("/tmp/ic_pose.json")


# -------------------- Utilities --------------------

def ensure_map_dir() -> Path:
    """Ensure the maps directory exists and return it."""
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    return MAPS_DIR


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def _ros2_available() -> bool:
    """Return True iff 'ros2' CLI is available in PATH."""
    return shutil.which("ros2") is not None


def _try_ros2_serialize(out_stem: Path) -> None:
    """
    Try to call slam_toolbox's serialize service to save the current map/pose graph.
    Adjust the service name below if your namespace differs.
    """
    # Common service names (some setups use leading /, some don't)
    candidates = [
        "/slam_toolbox/serialize_map",
        "slam_toolbox/serialize_map",
        "/serialize_map",
        "serialize_map",
    ]

    # Prefer the official service type
    service_type = "slam_toolbox/srv/SerializePoseGraph"

    last_err: Optional[Exception] = None
    for svc in candidates:
        try:
            # ros2 service call <name> <type> "filename: '<path>'"
            cmd = [
                "ros2", "service", "call",
                svc,
                service_type,
                f"filename: '{str(out_stem)}'"
            ]
            # Use check=True so a nonzero exit raises CalledProcessError
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"[slam] serialize_map OK via service '{svc}' → {out_stem}")
            return
        except Exception as e:
            last_err = e

    # If we reach here, all candidates failed
    raise RuntimeError(f"serialize_map service not available (last error: {last_err})")


# -------------------- Public API --------------------

def start_slam() -> subprocess.Popen:
    """
    Launch slam_toolbox (online_async) and return the Popen handle.
    This assumes a ROS2 environment is already sourced in the current shell.

    If you use a different launch file or namespace, adjust LAUNCH_CMD below.
    """
    # Example launch (most common):
    LAUNCH_CMD = ["ros2", "launch", "slam_toolbox", "online_async_launch.py"]

    # Start the process detached from our stdio (quiet by default).
    # If you want logs in your terminal, remove stdout/stderr redirection.
    try:
        proc = subprocess.Popen(
            LAUNCH_CMD,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp  # avoid killing SLAM if Python exits abruptly
        )
        print("[slam] Starting SLAM (slam_toolbox online_async)...")
    except FileNotFoundError as e:
        # ros2 not found
        raise RuntimeError(
            "ROS2 not found. Ensure your environment is sourced (e.g., 'source /opt/ros/<distro>/setup.bash') "
            "and slam_toolbox is installed."
        ) from e

    # Give slam_toolbox a moment to come up; adjust if needed
    time.sleep(2.0)
    return proc


def save_map(slam_handle: Any = None, name: Optional[str] = None) -> Path:
    """
    Save/serialize the current map. The 'slam_handle' is typically a subprocess.Popen,
    but we never treat it like a path.

    Returns the base output stem Path (without extension); slam_toolbox typically
    writes multiple files with this stem.

    On failure to call ROS2 serialize service, a small stub file is written instead
    to keep the pipeline from crashing.
    """
    maps_dir = ensure_map_dir()
    stem = name or f"map_{_timestamp()}"
    out_stem = maps_dir / stem  # e.g., maps/map_20250101_120102

    # If ROS2 CLI isn't available, write a harmless stub and return
    if not _ros2_available():
        stub = out_stem.with_suffix(".txt")
        stub.write_text(f"[slam] ROS2 not available; wrote stub at {time.ctime()}\n")
        print(f"[slam] ros2 CLI not found; wrote stub {stub}")
        return out_stem

    # Try the serialize service
    try:
        _try_ros2_serialize(out_stem)
        return out_stem
    except Exception as e:
        # Fallback: write stub so callers don't crash
        stub = out_stem.with_suffix(".txt")
        stub.write_text(
            f"[slam] serialize_map unavailable ({e}); stub written at {time.ctime()}\n"
            f"handle_type={type(slam_handle).__name__}\n"
        )
        print(f"[slam] serialize_map unavailable; wrote {stub}")
        return out_stem


def load_map(name: str) -> Path:
    """
    (Optional) Provide your load/deserialize behavior here.
    For now, this just returns the expected stem path inside maps/.
    """
    maps_dir = ensure_map_dir()
    stem = maps_dir / name
    print(f"[slam] load_map stub → {stem}")
    return stem


# def _get_current_pose_from_lidar() -> Tuple[float, float, float]:
#     """
#     Stub: return (x, y, theta). Extend to read TF or your own pose provider.
#     """
#     # Replace with actual TF/pose graph query if available
#     return (0.0, 0.0, 0.0)

def _get_current_pose_from_lidar():
    """
    Return (x, y, theta) from the running LiDAR pose thread.
    """
    try:
        from perception.lidar import get_pose
        return get_pose()  # thread-safe; returns latest pose
    except Exception:
        return (0.0, 0.0, 0.0)


# def teach_place(name: str, places_path: Path | str = "navigation/places.json") -> None:
#     """
#     Stub: capture current pose under a human-friendly name and store it.
#     Integrate with your navigation.Places class if you already have one.
#     """
#     try:
#         from navigation.places import Places  # your repo's Places class
#         pman = Places(places_path)
#         x, y, th = _get_current_pose_from_lidar()
#         pman.add_place(name, {"x": x, "y": y, "theta": th})
#         pman.save()
#         print(f"[slam] teach_place: '{name}' recorded at ({x:.2f}, {y:.2f}, {th:.2f})")
#     except Exception as e:
#         print(f"[slam] teach_place stub fallback ({e}); no place recorded.")

def teach_place(name: str, places_path: Path | str = "navigation/places.json") -> None:
    try:
        # CHANGE import
        from navigation.places import PlaceManager
        pm = PlaceManager(str(places_path))
        x, y, th = _get_current_pose_from_lidar()
        # Build the small pose record your PlaceManager expects
        from types import SimpleNamespace
        pose = SimpleNamespace(x=x, y=y, theta=th)
        pm.add_place(name, pose, aliases=[])
        print(f"[slam] teach_place: '{name}' recorded at ({x:.3f}, {y:.3f}, {th:.3f})")
    except Exception as e:
        print(f"[slam] teach_place stub fallback ({e}); no place recorded.")
