#!/usr/bin/env python3
"""
aQuatonomous: thruster driver with API or CLI fallback

Usage
1) Start the sim and set env in every terminal:
   export IGN_TRANSPORT_PARTITION=aquatonomous
   export IGN_TRANSPORT_TOPIC_PARTITION=aquatonomous
   export IGN_IP=127.0.0.1

2) Run:
   python3 Thrust_Control.py
"""

import os, time, subprocess, shutil
from dataclasses import dataclass
from typing import Callable, Tuple, List

# ---------- user config ----------
TOPIC_LEFT  = "/model/nd_boat/joint/left_prop_joint/cmd_thrust"
TOPIC_RIGHT = "/model/nd_boat/joint/right_prop_joint/cmd_thrust"
RATE_HZ     = 20.0
SAFETY_ZERO_TIME = 1.0

PLAN = [
    ("hold",  dict(left=200.0, right=200.0, duration=3.0)),
    ("hold",  dict(left=0.0,   right=0.0,   duration=1.0)),
    ("spin",  dict(thrust=150.0, duration=2.0)),
    ("ramp",  dict(start=0.0, end=300.0, duration=3.0)),
]

def my_custom_profile(t: float) -> Tuple[float, float]:
    import math
    thrust = 200.0 + 100.0 * math.sin(2.0 * math.pi * 0.5 * t)
    return thrust, thrust
# ---------------------------------

# Try to load transport; else mark as None to use CLI
DoubleMsg = None
transport = None
try:
    import gz.transport as transport
    try:
        from gz.msgs.double_pb2 import Double as DoubleMsg
    except Exception:
        from ignition.msgs.double_pb2 import Double as DoubleMsg  # type: ignore
except Exception:
    try:
        import ignition.transport as transport  # type: ignore
        from ignition.msgs.double_pb2 import Double as DoubleMsg  # type: ignore
    except Exception:
        transport = None
        DoubleMsg = None

def _cli_publish(topic: str, value: float):
    # Uses ign topic CLI
    return subprocess.run(
        ["ign", "topic", "-t", topic, "-m", "ignition.msgs.Double", "-p", f"data: {float(value)}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode == 0

@dataclass
class ThrustDriver:
    topic_left: str
    topic_right: str
    rate_hz: float
    use_cli: bool = False

    def __post_init__(self):
        self.dt = 1.0 / max(self.rate_hz, 1e-3)
        if transport and DoubleMsg:
            self.node = transport.Node()
            self.use_cli = False
        else:
            # No Python API, fall back to CLI
            if not shutil.which("ign"):
                raise RuntimeError("Neither gz/ign Python transport nor ign CLI found.")
            self.use_cli = True
            self.node = None

    def _publish_pair(self, left: float, right: float):
        if not self.use_cli:
            mL = DoubleMsg(); mL.data = float(left)
            mR = DoubleMsg(); mR.data = float(right)
            self.node.Publish(self.topic_left, mL)
            self.node.Publish(self.topic_right, mR)
        else:
            _cli_publish(self.topic_left, left)
            _cli_publish(self.topic_right, right)

    def hold(self, left: float, right: float, duration: float):
        t0 = time.time()
        while time.time() - t0 < duration:
            self._publish_pair(left, right)
            time.sleep(self.dt)

    def spin(self, thrust: float, duration: float):
        self.hold(+thrust, -thrust, duration)

    def ramp(self, start: float, end: float, duration: float):
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= duration:
                break
            alpha = t / max(duration, 1e-6)
            val = start + (end - start) * alpha
            self._publish_pair(val, val)
            time.sleep(self.dt)

    def custom(self, func: Callable[[float], Tuple[float, float]], duration: float):
        t0 = time.time()
        while True:
            t = time.time() - t0
            if t >= duration:
                break
            left, right = func(t)
            self._publish_pair(left, right)
            time.sleep(self.dt)

    def zero_for(self, duration: float):
        t0 = time.time()
        while time.time() - t0 < duration:
            self._publish_pair(0.0, 0.0)
            time.sleep(self.dt)

def main():
    print(f"Using topics:\n  L: {TOPIC_LEFT}\n  R: {TOPIC_RIGHT}")
    print(f"Rate: {RATE_HZ} Hz")
    print("Plan:", PLAN)
    print("Transport mode:", "Python API" if transport else "CLI fallback")

    print(f"Partition: {os.environ.get('IGN_TRANSPORT_PARTITION')}  "
          f"TopicPartition: {os.environ.get('IGN_TRANSPORT_TOPIC_PARTITION')}  "
          f"IP: {os.environ.get('IGN_IP')}")

    driver = ThrustDriver(TOPIC_LEFT, TOPIC_RIGHT, RATE_HZ)

    try:
        for mode, args in PLAN:
            print(f"\nExecuting: {mode} {args}")
            if mode == "hold":
                driver.hold(**args)
            elif mode == "spin":
                driver.spin(**args)
            elif mode == "ramp":
                driver.ramp(**args)
            elif mode == "custom":
                driver.custom(**args)
            else:
                print(f"Unknown mode {mode}, skipping")
        print("\nPlan complete. Zeroing thrusters...")
    except KeyboardInterrupt:
        print("\nInterrupted. Zeroing thrusters...")
    finally:
        driver.zero_for(SAFETY_ZERO_TIME)
        print("Done.")

if __name__ == "__main__":
    main()
