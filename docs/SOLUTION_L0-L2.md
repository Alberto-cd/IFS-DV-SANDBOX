# A possible solution: Levels 0 to 2

One way to do the first three levels of [LEVELS.md](LEVELS.md), from start
to finish. It's **one** solution, not the only one: read it when you're
stuck, or after you've tried yourself.

Everything below uses the package name `alberto`. Writing your own? Create
your package with `sb-new <you>` and replace `alberto` with your name in the
commands below.

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
