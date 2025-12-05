# Local Planner

In this file there is an explanation and instructions about the current progress of the local planner. 

## Table of Contents

- [Trajectory Generator Node](#trajectory-generator-node-trajectory_generator)
- [Local Planner Node](#local-planner-node-mpc_controller)
  - [Controller Selection](#controller-selection)
  - [Dummy Controller (Baseline)](#dummy-controller-baseline)
- [MPC Local Planner (Kinematic MPC Baseline)](#mpc-local-planner-kinematic-mpc-baseline)
  - [Model Used (TIAGo Base Kinematics)](#model-used-tiago-base-kinematics)
  - [MPC Formulation](#mpc-formulation)
  - [Solver and Implementation Details](#solver-and-implementation-details)
- [Updated Workspace Structure (Extended)](#updated-workspace-structure-extended)
- [Launch Files and Demos](#launch-files-and-demos)
  - [Local Path-Following Demo](#local-path-following-demo-global-path--local-planner)
  - [Launching the Demo](#launching-the-demo)
- [Known Limitations / TODOs](#known-limitations-for-now--todos)
  - [Tuning Tips](#tuning-tips)

## Trajectory Generator Node (`trajectory_generator`)

The `TrajectoryGenerator` node builds a geometric path and publishes it as a `nav_msgs/Path` on the `reference_path` topic.

**Topic:**

- Publishes: `reference_path` (`nav_msgs/Path`)

**Main parameters:**

- `path_type`: `'line'` or `'circle'`
- `start_x`, `start_y`: start point for line path
- `goal_x`, `goal_y`: goal point for line path
- `circle_center_x`, `circle_center_y`, `circle_radius`: circle path parameters
- `start_angle`: starting angle along the circle (rad)
- `direction`: `'ccw'` or `'cw'`
- `num_points`: number of waypoints before optional resampling
- `resample_to`: if > 0, resamples path to this number of points
- `frame_id`: frame for poses in the path (e.g. `odom` or `map`)
- `publish_rate`: Hz at which the path is republished

**Usage example (line path):**

```bash
ros2 run pdm_test trajectory_generator
--ros-args
-p path_type:=line
-p start_x:=0.0 -p start_y:=0.0
-p goal_x:=3.0 -p goal_y:=0.0
-p frame_id:=odom
-p publish_rate:=1.0
```

**Usage example (circle path):**

```bash
ros2 run pdm_test trajectory_generator
--ros-args
-p path_type:=circle
-p circle_center_x:=0.0 -p circle_center_y:=0.0
-p circle_radius:=1.0
-p start_angle:=0.0
-p direction:=ccw
-p frame_id:=odom
```


Internally, the node:

- Uses `PathGenerator` and `resample_path` from `pdm_test.utils`
- Builds an $(N, 3)$ numpy array of $[x, y, \theta]$
- Converts it into `nav_msgs/Path` with each point as a `geometry_msgs/PoseStamped`


## Local Planner Node (`mpc_controller`)

The `MpcController` node implements the local planner. It:

- **Subscribes to:**
  - `/mobile_base_controller/odom` (`nav_msgs/Odometry`) – robot pose
  - `/reference_path` (`nav_msgs/Path`) – global reference trajectory
- **Publishes:**
  - `/cmd_vel` (`geometry_msgs/Twist`) – base velocity commands

Two controller modes are available, selected via `controller_type`.

### Controller selection

**Parameter:**

- `controller_type`:
  - `dummy`: proportional heading controller with constant forward speed
  - `mpc`: model predictive controller (kinematic MPC baseline)

**Examples:**

```bash
ros2 launch pdm_test local_plan_baseline_demo.launch.py controller_type:=dummy
ros2 launch pdm_test local_plan_baseline_demo.launch.py controller_type:=mpc

ros2 run pdm_test mpc_controller --ros-args -p controller_type:=dummy
```



### Dummy controller (baseline)

The dummy controller provides a simple baseline:

- Finds the nearest point on the reference path to the current $(x, y)$
- Uses the stored heading $\theta$ at that point as the desired heading
- Computes the wrapped heading error:
  - $e_\theta = \theta_{\text{ref}} - \theta$, wrapped to $[-\pi, \pi]$
- Applies proportional control:
  - $\omega = k_{\text{heading}} \, e_\theta$
  - $v = v_{\text{const}}$ (constant speed)

**Parameters:**

- `k_heading`: proportional gain on heading error
- `v_const`: constant linear speed
- `control_rate`: control loop rate [Hz]
- `max_v`, `max_omega`: saturation limits (can be enforced by enabling clipping in the code)

This mode is useful to verify topic wiring and to compare against the MPC controller.


## MPC Local Planner (Kinematic MPC Baseline)

In `mpc` mode, the node runs a kinematic model predictive controller.

### Model used (TIAGo base kinematics)

The MPC uses a kinematic differential-drive model implemented in `TiagoDifferentialDriveModel`.

**State:**

- $x = [x, y, \theta]^\top$ – planar position and heading

**Control:**

- $u = [v, \omega]^\top$ – linear and angular velocity

**Continuous-time dynamics:**

- $\dot{x} = v \cos\theta$  
- $\dot{y} = v \sin\theta$  
- $\dot{\theta} = \omega$

**Discrete-time update** (Euler):

- $x_{k+1} = x_k + \dot{x}_k \, \Delta t$
- $y_{k+1} = y_k + \dot{y}_k \, \Delta t$
- $\theta_{k+1} = \text{wrap\_to\_pi}(\theta_k + \dot{\theta}_k \, \Delta t)$

The model class provides:

- `continuous_dynamics(state, control)` – returns $[\dot{x}, \dot{y}, \dot{\theta}]$
- `discrete_step(state, control)` – one-step prediction
- `simulate_trajectory(initial_state, control_sequence)` – rollout over a control sequence

### MPC formulation

The controller solves a finite-horizon problem at each control step.

**Prediction horizon:**

- `mpc_horizon = N` steps (default 10)
- `dt` (default 0.1 s)
- Lookahead: $N \cdot dt \approx 1.0$ s

**Reference extraction:**

- Find nearest path point to current $(x, y)$
- Take the next `N` points of the path
- If fewer than `N` remain, pad with the final path point
- Result is an $(N, 3)$ reference array $[x_{\text{ref}}, y_{\text{ref}}, \theta_{\text{ref}}]$

**Cost function** (no terminal cost):

For predicted states $x_k$ and reference $x^{\text{ref}}_k$,

$$
J = \sum_{k=0}^{N-1} \left(
Q_x (x_k - x_k^{\text{ref}})^2 +
Q_y (y_k - y_k^{\text{ref}})^2 +
Q_\theta (\theta_k - \theta_k^{\text{ref}})^2 +
R_v v_k^2 +
R_\omega \omega_k^2
\right)
$$

with heading error wrapped to $[-\pi, \pi]$.

**Default weights (tunable):**

- `Q_x = 10.0`, `Q_y = 10.0`, `Q_theta = 5.0`
- `R_v = 0.1`, `R_omega = 0.1`

**Constraints:**

- `v_min <= v_k <= max_v`
- `-max_omega <= omega_k <= max_omega`

Typical values:

- `v_min`: around `-0.3` to `-0.5`
- `max_v`: around `0.5`–`1.0`
- `max_omega`: around `1.0`

Obstacle avoidance is not included in this baseline; the controller is focused on path tracking.

### Solver and implementation details

- Implemented with `scipy.optimize.minimize` (SLSQP)
- Decision variables: flattened control sequence of length `2 * mpc_horizon`:
  - $[v_0, \omega_0, v_1, \omega_1, \dots, v_{N-1}, \omega_{N-1}]$
- Initial guess: constant forward velocity `v_const`, zero angular rate
- Box constraints applied per control element
- Cost evaluated by rolling out the kinematic model using `simulate_trajectory`
- Only the first control pair $(v_0, \omega_0)$ is applied (receding horizon)





## Updated Workspace Structure (extended)

The workspace now includes the new trajectory generator, MPC controller, and model utilities:

```
src/pdm_test/
├── launch/
│ ├── cafe.launch.py
│ └── local_plan_baseline_demo.launch.py # Global path + local planner demo
├── pdm_test/
│ ├── init.py
│ ├── straight_driver.py # Legacy straight-line driver
│ ├── rrc_server.py
│ ├── trajectory_generator.py # Publishes /reference_path (nav_msgs/Path)
│ ├── mpc_controller.py # Local planner (dummy + MPC modes)
│ ├── models/
│ │ └── tiago_diff_drive_model.py # Kinematic differential-drive model for TIAGo
│ └── utils.py # PathGenerator, resampling, helpers
├── worlds/
│ ├── cafe.world
│ ├── cafe_table.world
│ ├── cafe_dynamic.world
│ └── empty.world
├── package.xml
├── setup.py
└── setup.cfg
```

# Launch Files and Demos

## Local Path-Following Demo (Global Path + Local Planner)

A new demo adds a **global path generator** and a **local planner** (dummy controller or MPC) to replace the older straight driver.

This demo launches:

- TIAGo simulation in Gazebo (via `cafe.launch.py`)
- A `trajectory_generator` node that publishes a reference path as `nav_msgs/Path` on `/reference_path`
- An `mpc_controller` node that consumes `/reference_path` and `/mobile_base_controller/odom` and publishes `/cmd_vel`
- RViz for visualization

### Launching the demo

#### Dummy local planner (baseline)
```bash
ros2 launch pdm_test local_plan_baseline_demo.launch.py controller_type:=dummy
```

#### MPC local planner (optimization-based)
```bash
ros2 launch pdm_test local_plan_baseline_demo.launch.py controller_type:=mpc
```


**Launch arguments:**

- `world_name` (string, default: `empty`): Gazebo world to load (same as `cafe.launch.py`)
- `path_type` (string, default: `circle`): `line` or `circle` reference path
- `k_heading` (float, default: `5.0`): heading gain for dummy controller
- `v_const` (float, default: `0.3`): constant forward speed for dummy controller
- `controller_type` (string, default: `dummy`): `dummy` or `mpc`



# Known Limitations For Now / TODOs

- MPC does not include obstacle avoidance (path tracking only)
- No terminal cost in cost function yet
- Solver is pure Python (`scipy.optimize`), may be slow for larger horizons
- Control saturation is commented out by default

## Tuning Tips

- Increase `Q_x`, `Q_y` for tighter path tracking
- Increase `Q_theta` if heading oscillates
- Increase `R_v`, `R_omega` to smooth control inputs
- Reduce `mpc_horizon` if solver is too slow
- Ensure `dt` matches your `control_rate` (e.g., `dt = 1/control_rate`)
