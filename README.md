# PDM TIAGo Workspace

Motion planning and decision-making for TIAGo mobile manipulator in ROS 2 Humble.

---

## Authors
TU Delft MSc Robotics students:
* Lapo Veca (5679818), 
* Nikhil Krishnapura Gopala Krishna (6434231),
* Raaf ter Woerds (5368537), 
* Ricardo Steen (5091969)


## Quick Setup

**Prerequisites:** ROS 2 Humble + `~/tiago_public_ws` built, and casadi installed.

Set up the `~/tiago_public_ws` repository, following the installation instructions from: https://github.com/pal-robotics/tiago_simulation/tree/humble-devel.

**Setting Up Custom Worlds:**

The PDM workspace includes 8 custom Gazebo worlds. To make them available to TIAGo:

```bash
# Copy all custom worlds to tiago_public_ws
cp ~/PDM_tiago_ws/src/pdm_test/worlds/*.world ~/tiago_public_ws/src/pal_gazebo_worlds/worlds/

# Rebuild and source
cd ~/tiago_public_ws
colcon build --symlink-install
source install/setup.bash
```

Once this is done, clone and build the project's workspace.

```bash
cd ~
git clone git@github.com:raaftw/PDM_tiago_ws.git
cd PDM_tiago_ws
colcon build --symlink-install
source install/setup.bash
```

**Always source in new terminals:**
```bash
source /usr/share/gazebo/setup.bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source ~/PDM_tiago_ws/install/setup.bash
```

---

## Running the System

### Scenario 1: Base Navigation (MPC + RRT*)

Full MPC + global planner system:

```bash
ros2 launch pdm_test mpc_combined.launch.py world_name:=walls_blocks
```

Then in RViz:
1. Click **"Nav2 Goal"** button
2. Click goal location on map (note: if goal is very close to obstacles, the path will be rejected)
3. RRT* plans → MPC executes

**Available worlds:** `cafe`, `cafe_table`, `walls`, `walls_blocks`, `wiping_env`, `wiping_env_small_table`

### Scenario 2: Table Detection + Arm Manipulation

Detect tables and trigger arm wiping:

```bash
# Terminal 1: Launch Gazebo
ros2 launch pdm_test cafe.launch.py world_name:=wiping_env_small_table

# Terminal 2: Detect tables
ros2 launch pdm_test table_detector.launch.py \
  occupied_threshold:=65 \
  min_table_area:=0.15 \
  max_table_area:=1.0 \
  erode_iterations:=0 \
  save_to_yaml:=True

# Terminal 3: Arm executor (RRT-Connect + IK/FK)
ros2 run pdm_test tiago_table_cleaner_rrt_visualization_ik

# Terminal 4: Trigger arm
ros2 service call /clean_table std_srvs/srv/Trigger
```

---

## Core Components

### Base Navigation

| File | Purpose |
|------|---------|
| `local_planner.py` | MPC controller: trajectory tracking + obstacle avoidance (10 Hz) |
| `global_planner.py` | RRT* path planner: collision-free optimal paths |
| `ground_truth_republisher.py` | Convert Gazebo odom → PoseStamped |
| `obstacle_publisher.py` | Extract obstacles from costmap |
| `models/tiago_diff_drive_model.py` | Differential drive kinematics |

### Arm Manipulation

| File | Purpose |
|------|---------|
| `rrt_connect_2.py` | RRT-Connect planner |
| `collision_checker.py` | Joint limits + self-collision checks |
| `tiago_table_cleaner_rrt_visualization_ik.py` | Main arm executor: RRT-Connect + IK/FK + metrics |


---

## Project Structure

```
src/pdm_test/
├── README.md
├── package.xml
├── setup.py
├── setup.cfg
│
├── pdm_test/
│   ├── local_planner.py              # MPC base controller
│   ├── global_planner.py             # RRT* global planner
│   ├── ground_truth_republisher.py   # Odometry conversion
│   ├── obstacle_publisher.py         # Costmap → obstacles
│   │
│   ├── rrt_connect_2.py              # RRT-Connect extended
│   ├── collision_checker.py          # Joint limits + collision
│   ├── tiago_table_cleaner_rrt_visualization_ik.py   # Main arm executor
│   │
│   ├── models/
│   │   └── tiago_diff_drive_model.py # Kinematics
├── launch/
│   ├── mpc_combined.launch.py        # Full base navigation ★
│   ├── cafe.launch.py                # Gazebo + TIAGo base
│   ├── table_detector.launch.py      # Table detection
│   ├── global_planner_demo.launch.xml
│   ├── local_planner_demo.launch.py
│   └── *.rviz                        # RViz configs
│
├── worlds/
│   ├── cafe.world
│   ├── cafe_table.world
│   ├── walls.world
│   ├── walls_blocks.world
│   ├── wiping_env.world
│   └── wiping_env_small_table.world
│
├── maps/
│   ├── cafe_map.yaml / .pgm
│   ├── walls_map.yaml / .pgm
│   ├── walls_blocks_map.yaml / .pgm
│   ├── cafe_table_map.yaml / .pgm
│   └── wiping_env_*.yaml / .pgm
│
└── test/
```

---

## ROS Topics & Services

### Base Navigation

| Topic | Type | Node | Direction |
|-------|------|------|-----------|
| `/ground_truth_odom` | Odometry | Gazebo | → MPC |
| `/scan_raw` | LaserScan | Gazebo | → MPC |
| `/map` | OccupancyGrid | Nav2 map_server | → RRT*, Table Detector |
| `/reference_path` | Path | RRT* | → MPC |
| `/cmd_vel` | Twist | MPC | → Base |

### Arm Manipulation

| Topic | Type | Node | Direction |
|-------|------|------|-----------|
| `/detected_tables` | PoseArray | Table Detector | → Visualization |
| `/table_markers` | MarkerArray | Table Detector | → RViz |
| `/arm_controller/joint_trajectory` | JointTrajectory | Arm executor | → Arm |
| `/joint_states` | JointState | Gazebo | → Arm executor |
| `/rrt_debug_markers` | MarkerArray | Arm executor | → RViz (debug) |
| `/arm_planner_metrics` | String (JSON) | Arm executor | → Log |

### Services

| Service | Type | Server | Purpose |
|---------|------|--------|---------|
| `/clean_table` | Trigger | Arm executor | Trigger arm manipulation |
| `/compute_ik` | GetPositionIK | MoveitMsgs | Inverse kinematics |
| `/compute_fk` | GetPositionFK | MoveitMsgs | Forward kinematics |

---

## Configuration

### MPC Controller

Edit launch parameters or `local_planner.py`:
- `Q_x`, `Q_y` (35.0): Position tracking weight
- `Q_f_x`, `Q_f_y` (80.0): Terminal cost (goal accuracy)
- `R_v`, `R_omega` (0.8, 0.02): Control effort
- `brake_weight` (25.0): Speed penalty near goal
- `d_safe` (0.35m): Hard obstacle constraint
- `d_preferred` (0.7m): Soft obstacle penalty
- `max_v` (0.5 m/s): Max linear velocity
- `max_omega` (0.5 rad/s): Max angular velocity

### RRT* Global Planner

Edit `global_planner.py`:
```python
max_iterations = 1000
step_size = 1.0
rewire_radius = 3.0
goal_sample_rate = 0.3
```

### Table Detector

Launch parameters:
```bash
ros2 launch pdm_test table_detector.launch.py \
  occupied_threshold:=65 \       # Occupancy threshold (higher → stricter)
  min_table_area:=0.15 \         # Min table size (m²)
  max_table_area:=1.0 \          # Max table size (m²)
  erode_iterations:=0 \          # Morphology: 0=off, 1+=shrink obstacles
  save_to_yaml:=True
```

**Tuning:**
- No detections: lower `occupied_threshold` or set `erode_iterations:=0`
- Tables too big: set `erode_iterations:=1` or raise `occupied_threshold`
- Wrong count: adjust `min_table_area` / `max_table_area`

### Arm RRT-Connect

Edit `tiago_table_cleaner_rrt_visualization_ik.py` or `rrt_connect.py`:
```python
step_size = 0.2              # Joint space step (radians)
max_iters = 2000             # Max planning iterations
goal_bias = 0.5              # Goal sampling probability
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| MPC solver fails | Reduce `mpc_horizon`, loosen `d_safe`, check obstacle count |
| Robot too slow | Lower `Q_x`, `Q_y` to 20–30 |
| Robot overshoots | Increase `Q_f_x`, `Q_f_y` to 80–100 |
| MPC ignores walls | Expand FOV filter in `_laser_scan_cb()` |
| No table detections | Lower `occupied_threshold` or set `erode_iterations:=0` |
| Arm planner fails | Check IK/FK service, lower `step_size`, increase `max_iters` |
| Package not found | Source all workspaces in order: Gazebo → ROS 2 → TIAGo → PDM |

---

## Verify Installation

```bash
# Build
cd ~/PDM_tiago_ws
colcon build --symlink-install

# Check entry points
grep "console_scripts" setup.py

# Test nodes exist
ros2 run pdm_test mpc_controller --help
ros2 run pdm_test global_planner --help
ros2 launch pdm_test mpc_combined.launch.py --show-args
```

---

**Last Updated:** January 12, 2026  
