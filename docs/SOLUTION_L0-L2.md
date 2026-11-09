# A possible solution: Levels 0 to 2

One way to do the first three levels of [LEVELS.md](LEVELS.md), from start
to finish. It's **one** solution, not the only one: read it when you're
stuck, or after you've tried yourself.

The finished code is already in this repo, in
[`ros2_ws/src/alberto_pipeline`](../ros2_ws/src/alberto_pipeline). To run it:

```bash
sb-build
sb-launch sim:=mock perception:=gt control:=alberto_pipeline/control
```

Writing your own? Create your package with `sb-new <you>` and replace
`alberto` with your name in the commands below.

| Level | You change | File |
|---|---|---|
| 0 | nothing, you look and break things | none |
| 1 | `compute()` | `ros2_ws/src/alberto_pipeline/alberto_pipeline/control.py` |
| 2 | `plan()` | `ros2_ws/src/alberto_pipeline/alberto_pipeline/planning.py` |

---

## Level 0: drive it, look at it, break it

### 1–2. Run it and find each stage in Foxglove

```bash
sb-launch sim:=mock
```

Connect Foxglove to `ws://localhost:8765`.
In the 3D panel:

| What you see | Topic | Stage |
|---|---|---|
| point cloud | `/lidar_points/viz` | simulator |
| tall cones | `/sandbox/viz/cones` (from `/sandbox/cones`) | perception |
| short cones | `/sandbox/viz/map` (from `/sandbox/map`) | SLAM |
| grey cones | `/sandbox/viz/track` | ground truth |
| line in front of the car | `/sandbox/path` | planning |
| dot on the path | `/sandbox/control/lookahead` | control |

### 3. Nodes and rates

```bash
ros2 node list
ros2 topic hz /sandbox/odom      # repeat for every topic below
```

| Topic | Rate | Why |
|---|---|---|
| `/imu` | 400 Hz in IFSSIM (100 Hz on the mock) | a sensor: cheap to read, so it runs fast |
| `/sandbox/odom` | ~100 Hz | odometry adds up small steps from the IMU and wheels; small steps = small errors |
| `/control_command` | 40 Hz | the car must react quickly, even when the path hasn't changed |
| `/lidar_points`, `/sandbox/cones` | 10 Hz | the LiDAR turns 10 times a second; perception runs once per scan |
| `/sandbox/pose`, `/sandbox/map` | ~10 Hz | SLAM updates when new cones arrive |
| `/sandbox/path` | ~10 Hz | planning updates when the map changes |

**Fastest:** `/imu` (fastest of all) and `/sandbox/odom` (fastest of the
pipeline topics). Odometry integrates speed and turn rate over time, so the
smaller each time step, the less error builds up. Everything that depends on
the LiDAR can't go faster than 10 Hz.

### 4. Drive it yourself

```bash
sb-launch sim:=mock control:=off
ros2 run sandbox_tools teleop          # in a second shell
```

### 5. Break things

Predict first, then try it. Each experiment starts with a fresh launch:
stop the previous one with `Ctrl+C`. "Shell 2" means a second sandbox
shell (run `tools/shell.sh`, or `tools\shell.cmd` on Windows, in another
terminal).

#### a) Kill planning

```bash
# shell 1
sb-launch sim:=mock planning:=off
```

- **What happens:** the car never moves.
- **Why:** with no `/sandbox/path`, control has nothing to follow, so it
  brakes (`brake = 1.0`).

#### b) Perfect vs real odometry

```bash
# shell 1: perfect odometry (ground truth)
sb-launch sim:=mock odometry:=gt

# shell 2: watch the pose error
ros2 topic echo /sandbox/monitor/pose_error
```

Note the numbers, stop both with `Ctrl+C`, then do the same with the baseline odometry:

```bash
# shell 1: baseline odometry
sb-launch sim:=mock

# shell 2
ros2 topic echo /sandbox/monitor/pose_error
```

- **What happens:** with ground truth, the error stays at about 0. With the
  baseline it grows slowly as the car drives.
- **Why:** wheel speed and the gyro have small errors. Odometry adds them
  up, so the error only grows. SLAM corrects part of it using the cones.

#### c) Noisy perception

```bash
# shell 1
sb-launch sim:=mock perception:=gt

# shell 2, once the car is driving
ros2 param set /gt_cones position_noise 0.3
```

- **What happens:** cones in the map jump around, the path wiggles, the
  steering twitches, and maybe a cone gets hit.
- **Why:** noisy cones → noisy map → the path's midpoints move at every
  update → pure pursuit follows each wiggle.

#### d) Swapped colours

```bash
# shell 1
sb-launch sim:=mock perception:=gt

# shell 2, once the car is driving
ros2 param set /gt_cones swap_rate 0.2
```

- **What happens:** the path sometimes kinks for a moment, mostly at its
  far end.
- **Why:** a blue cone labelled yellow gets paired with a cone on the same
  edge, so a midpoint lands on the track edge. SLAM's colour vote fixes old
  cones, but new cones at the horizon have only 1–2 votes.

#### e) Go faster

```bash
# shell 1
sb-launch sim:=mock

# shell 2, once the car is driving
ros2 param set /control target_speed 9.0
```

- **What happens:** on the mock the lap is much faster (~42 s), but the car
  hits a cone and the steering gets jerky. In IFSSIM it slides wide in the
  bends.
- **Why:** the baseline never slows down for bends. Its lookahead is
  1 + 0.5 × 9 = 5.5 m, so it aims across the inside of the corner.

---

## Level 1: control

### The idea

The baseline has two problems:
1. **One constant speed (3 m/s).** It's slow on straights and still too fast
   for tight bends.
2. **Lookahead grows without limit**, so at speed it cuts corners.

The solution keeps pure pursuit for steering, but adds:

| Part | What it does |
|---|---|
| **Speed from the path's curvature** | Measure how much the path bends ahead. Tight bend → low speed: `v = sqrt(A_LAT / curvature)`. Then make sure we can brake in time: `v_now ≤ sqrt(v_bend² + 2 · A_BRAKE · distance)` |
| **Capped lookahead** | `Ld = 2 + 0.35·v`, at most 6 m |
| **Steering rate limit** | Steering can't jump between two ticks. Smoother steering, and less sliding in IFSSIM |
| **PI speed controller** | P alone never quite reaches the target speed. The I term removes that error |

### The code

Open `alberto_pipeline/control.py`. **Replace everything above `class Control(Node):`**
with this (leave the class and `main()` as they are):

```python
"""My CONTROL node.   sb-launch control:=alberto_pipeline/control

Pure pursuit for steering + speed profile from the path's curvature.
"""
import math

import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body

WHEELBASE = 1.627                 # m
MAX_STEER = math.radians(22.4)    # steering = 1.0 means this angle at the wheels
DT = 1.0 / 40.0                   # tick() runs at 40 Hz

# Tuning (start here, then tune in IFSSIM)
V_MAX = 6.0          # m/s on the straights
A_LAT = 3.0          # m/s², max sideways acceleration in a bend (lower = slower corners)
A_BRAKE = 2.5        # m/s², how hard we plan to slow down before a bend
LD_MIN = 2.0         # m, lookahead at standstill
LD_GAIN = 0.35       # s, extra lookahead per m/s
LD_MAX = 6.0         # m
STEER_RATE = 2.5     # max change of the steering command per second (full lock in 0.4 s)
KP, KI = 0.4, 0.15   # speed PI controller

_state = {"steer": 0.0, "i": 0.0}   # remembered between ticks


def resample(path, step=0.25):
    """Same path, points every `step` metres (the planner's points can be far apart)."""
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    if s[-1] < step:
        return path
    t = np.arange(0.0, s[-1], step)
    return np.column_stack((np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])))


def point_at_distance(path, ld):
    """First path point at least `ld` metres from the car and in front of it."""
    d = np.hypot(path[:, 0], path[:, 1])
    ok = np.flatnonzero((d >= ld) & (path[:, 0] > 0.0))
    return path[ok[0]] if len(ok) else path[-1]


def curvature(path, gap=6):
    """|curvature| (1/m) at each point, from the circle through 3 points `gap` apart."""
    k = np.zeros(len(path))
    for i in range(gap, len(path) - gap):
        a, b, c = path[i - gap], path[i], path[i + gap]
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        abc = np.linalg.norm(b - a) * np.linalg.norm(c - b) * np.linalg.norm(c - a)
        k[i] = 2.0 * area2 / max(abc, 1e-6)
    return k


def target_speed(path):
    """Fastest speed now that still lets us slow down in time for every bend ahead."""
    k = curvature(path)
    v_bend = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(k, 1e-3)))
    s = np.arange(len(path)) * 0.25                         # distance to each point
    return float(np.min(np.sqrt(v_bend ** 2 + 2.0 * A_BRAKE * s)))


def compute(path_body: np.ndarray, speed: float):
    """Path in the car frame + current speed -> (throttle, steering, brake). steering + = left."""
    if len(path_body) < 2:
        _state["i"] = 0.0
        return 0.0, 0.0, 1.0
    path = resample(path_body)

    # Lateral: pure pursuit, lookahead grows with speed
    ld = min(LD_MIN + LD_GAIN * speed, LD_MAX)
    tx, ty = point_at_distance(path, ld)
    alpha = math.atan2(ty, tx)
    kappa = 2.0 * math.sin(alpha) / max(math.hypot(tx, ty), 0.1)
    steer = float(np.clip(math.atan(WHEELBASE * kappa) / MAX_STEER, -1.0, 1.0))
    # Rate limit: no jumps when the path flickers between two planner updates
    step = STEER_RATE * DT
    steer = _state["steer"] + float(np.clip(steer - _state["steer"], -step, step))
    _state["steer"] = steer

    # Longitudinal: speed profile from the path's curvature + PI on speed
    v_ref = target_speed(path)
    err = v_ref - speed
    _state["i"] = float(np.clip(_state["i"] + err * DT, -2.0, 2.0))
    u = KP * err + KI * _state["i"]
    throttle = float(np.clip(u, 0.0, 1.0))
    brake = float(np.clip(-u, 0.0, 1.0)) if err < -0.3 else 0.0
    return throttle, steer, brake
```

### Run and score it

```bash
sb-build
sb-launch sim:=mock perception:=gt control:=alberto_pipeline/control          # watch it
sb-score perception:=gt control:=alberto_pipeline/control --scenario mock-B   # quick check
sb-score perception:=gt control:=alberto_pipeline/control --scenario ifssim-B # the test that counts
```

**Done when:** one lap of track B in IFSSIM, 0 cones hit, faster than
118.8 s. Then try `--scenario ifssim-A` (beat 176.4 s).

### Tuning in IFSSIM

The mock has unlimited grip; IFSSIM doesn't. If the car slides wide in the
bends in IFSSIM:

| Symptom | Change |
|---|---|
| Slides wide in bends | Lower `A_LAT` (try 2.0, then 1.5) |
| Runs into the bend too fast | Lower `A_BRAKE` (brake earlier) |
| Cuts the inside of hairpins | Lower `LD_MAX` or `LD_GAIN` |
| Wobbles left-right on straights | Raise `LD_MIN`, or lower `STEER_RATE` |
| Slow on straights | Raise `V_MAX` |

Change **one** value at a time and score again.

### The experiments, answered

- **Lookahead vs speed.** Short lookahead at high speed → oscillates
  (it overcorrects). Long lookahead → cuts corners (it aims across the
  inside). That's why the lookahead grows with speed and has a cap.
- **Slow down where the path bends.** Curvature = 1 / radius of the bend.
  Measure it from 3 points on the path (`curvature()` above), then
  `v = sqrt(A_LAT / curvature)` keeps the sideways acceleration under `A_LAT`.
- **`position_noise` on.** The path wiggles → the steering twitches.
  The steering rate limit and a longer lookahead both smooth it out.
