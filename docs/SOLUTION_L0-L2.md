# A possible solution: Levels 0 to 2

One way to do the first three levels of [LEVELS.md](LEVELS.md), from start
to finish. It's **one** solution, not the only one: read it when you're
stuck, or after you've tried yourself.

The finished code is already in this repo, in
[`ros2_ws/src/alberto_pipeline`](../ros2_ws/src/alberto_pipeline). To run it:

```bash
sb-build
sb-launch sim:=mock perception:=gt planning:=alberto_pipeline/planning control:=alberto_pipeline/control
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

---

## Level 2: planning

### The idea

The baseline's weak spots and the fixes:

| Weak spot | Fix |
|---|---|
| Trusts colours 100 %. No colour = no path | `UNKNOWN` cones get their side from geometry (`colour_by_side` in `sandbox_tools/geometry.py`) |
| Pairs only from blue to yellow | Pair from **both** edges → twice as many centre points, fewer gaps when cones are missed |
| One edge only: shifts sideways in `y` (wrong in a bend) | Shift **perpendicular to the edge**, half a track width inwards |
| No smoothing: one bad cone bends the path | Resample every 0.5 m and average neighbouring points |

The steps, in order:

1. Keep cones closer than 20 m and not far behind the car.
2. Give every cone a side: blue = left, yellow = right. Big orange cones:
   by which side of the car they're on. `UNKNOWN`: from geometry.
3. Order each edge in driving direction.
4. Centre points: for each cone, its nearest cone on the other edge
   (2–6 m away) → midpoint.
5. Chain the centre points from the car forward (like the baseline).
6. Smooth.

### The code

Open `alberto_pipeline/planning.py`. **Replace everything above `class Planning(Node):`**
with this (leave the class and `main()` as they are):

```python
"""My PLANNING node.   sb-launch planning:=alberto_pipeline/planning

Centre line between the two track edges, robust to missing colours.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, body_to_world, colour_by_side, cones_to_numpy, world_to_body

HORIZON = 20.0       # m, only use cones closer than this
HALF_WIDTH = 1.5     # m, half the track width
MAX_STEP = 6.0       # m, max gap between two centre points
MAX_TURN = math.radians(50.0)
STEP = 0.5           # m, spacing of the published path


def edge_normals(edge):
    """Unit vector pointing LEFT of the driving direction at each cone of an ordered edge."""
    if len(edge) < 2:
        return np.tile((0.0, 1.0), (len(edge), 1))
    t = np.gradient(edge, axis=0)
    t /= np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-9)[:, None]
    return np.column_stack((-t[:, 1], t[:, 0]))


def order_edge(edge):
    """Sort the cones of one edge in driving order: start nearest the car, walk to the nearest next one."""
    if len(edge) < 2:
        return edge
    left = list(range(len(edge)))
    cur = min(left, key=lambda i: math.hypot(*edge[i]) - 2.0 * (edge[i][0] < 0))   # behind first
    out = [cur]
    left.remove(cur)
    while left:
        nxt = min(left, key=lambda i: np.hypot(*(edge[i] - edge[cur])))
        if np.hypot(*(edge[nxt] - edge[cur])) > 8.0:
            break
        out.append(nxt)
        left.remove(nxt)
        cur = nxt
    return edge[out]


def centre_points(blue, yellow):
    """Candidate points on the centre line, from both edges."""
    pts = []
    for a, b in ((blue, yellow), (yellow, blue)):          # pair from both sides -> denser line
        for c in a:
            if len(b) == 0:
                break
            d = np.hypot(*(b - c).T)
            j = int(np.argmin(d))
            if 2.0 < d[j] < 6.0:                            # plausible track width
                pts.append((c + b[j]) / 2.0)
    if len(pts) >= 3:
        return np.array(pts)
    # One edge only (or a broken pairing): step half a track width inwards,
    # perpendicular to the edge (not just sideways in y: that's wrong in a bend).
    if len(blue) >= len(yellow) and len(blue):
        return blue - HALF_WIDTH * edge_normals(blue)
    if len(yellow):
        return yellow + HALF_WIDTH * edge_normals(yellow)
    return np.empty((0, 2))


def chain(mids):
    """From the car, keep jumping to the closest centre point ahead that doesn't turn too sharply."""
    path, heading, rest = [np.zeros(2)], 0.0, list(mids)
    while rest:
        cur = path[-1]
        best, best_d = None, MAX_STEP
        for k, m in enumerate(rest):
            v = m - cur
            d = math.hypot(*v)
            turn = abs((math.atan2(v[1], v[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
            if 0.5 < d < best_d and turn < MAX_TURN:
                best, best_d = k, d
        if best is None:
            break
        nxt = rest.pop(best)
        heading = math.atan2(nxt[1] - cur[1], nxt[0] - cur[0])
        path.append(nxt)
        # drop the leftovers right next to the point we just used (pairs from both sides)
        rest = [m for m in rest if math.hypot(*(m - nxt)) > 1.0]
    return np.array(path)


def smooth(path, window=5):
    """Resample every STEP metres and average neighbours. The first point stays at the car."""
    if len(path) < 3:
        return path
    seg = np.hypot(*np.diff(path, axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    t = np.arange(0.0, s[-1], STEP)
    p = np.column_stack((np.interp(t, s, path[:, 0]), np.interp(t, s, path[:, 1])))
    if len(p) <= window:
        return p
    k = np.ones(window) / window
    pad = np.vstack([p[:1]] * (window // 2) + [p] + [p[-1:]] * (window // 2))
    out = np.column_stack([np.convolve(pad[:, i], k, mode="valid") for i in (0, 1)])
    out[0] = (0.0, 0.0)
    return out


def plan(cones_body: np.ndarray, colors: np.ndarray) -> np.ndarray:
    """Cones in the car frame -> path points (K, 2) in the car frame, starting at (0, 0)."""
    straight = np.array([[0.0, 0.0], [3.0, 0.0]])
    if len(cones_body) == 0:
        return straight
    near = (np.hypot(cones_body[:, 0], cones_body[:, 1]) < HORIZON) & (cones_body[:, 0] > -3.0)
    xy, side = cones_body[near], colors[near].copy()
    if len(xy) < 2:
        return straight
    # Big orange gate cones sit on the edges too: left of the car = left edge.
    gate = side == Cone.ORANGE_BIG
    side[gate] = np.where(xy[gate, 1] > 0.0, Cone.BLUE, Cone.YELLOW)
    # No colour (UNKNOWN, small orange)? Guess the edge from geometry.
    unsure = (side != Cone.BLUE) & (side != Cone.YELLOW)
    if unsure.any():
        side[unsure] = colour_by_side(xy)[unsure]
    blue = order_edge(xy[side == Cone.BLUE])
    yellow = order_edge(xy[side == Cone.YELLOW])
    path = chain(centre_points(blue, yellow))
    return smooth(path) if len(path) >= 2 else straight
```

### Run and score it

Level 2 uses **your Level 1 control too**:

```bash
sb-build
sb-launch sim:=mock perception:=gt planning:=alberto_pipeline/planning control:=alberto_pipeline/control
sb-score perception:=gt planning:=alberto_pipeline/planning control:=alberto_pipeline/control --gt-noise --scenario ifssim-B
```

Put `--gt-noise` and `--scenario` **after** all the `stage:=...` arguments:
`sb-score` rejects stage arguments that come after a `--` option.

**Done when:** one lap of track B in IFSSIM with `--gt-noise`, 0 cones hit.
Then the same with `--scenario ifssim-A`.

### The experiments, answered

- **Your path vs the baseline's.** Yours is smoother (resampled + averaged)
  and denser. With your control, a smoother path means a lower steering
  rate. On the mock both get about the same Control score (88–89), so look
  at the steering in Foxglove to see the difference.
- **Other tracks.** `sim tracks` lists them; run
  `sb-launch sim:=mock track:=<name> perception:=gt planning:=alberto_pipeline/planning control:=alberto_pipeline/control`.
- **`unknown_rate 1.0` (no colours at all).** The baseline has no blue or
  yellow cones to pair, so its path is just the car's own position: the car
  stays stopped.
  This planner gives every cone a side from geometry and still drives.
  ```bash
  ros2 param set /gt_cones unknown_rate 1.0
  ```
