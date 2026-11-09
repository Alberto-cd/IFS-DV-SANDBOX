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
