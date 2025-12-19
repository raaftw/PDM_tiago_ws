# PDM TIAGo Workspace
**Motion planning and decision-making for TIAGo mobile manipulator in ROS 2 Humble.**

Overlay workspace on `~/tiago_public_ws`. Contains custom Gazebo worlds and driver algorithms (nodes) for the TIAGo robot.

---

## Quick Setup

**Prerequisites:** ROS 2 Humble + `~/tiago_public_ws` built.

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

## Running the Simulation

Default (empty world + circle driver):
```bash
ros2 launch pdm_test cafe.launch.py
```

Other worlds:
```bash
ros2 launch pdm_test cafe.launch.py world_name:=cafe
ros2 launch pdm_test cafe.launch.py world_name:=pal_office
```

---

## Test Scripts

### Circle Driving (already running in default launch)
The `straight_driver` node publishes `/cmd_vel` Twist messages (linear + angular velocity).

**Parameters:**
- `linear_speed` (m/s, default 0.2)
- `angular_speed` (rad/s, default 0.5)
- `duration` (s, default 60.0)

Run with custom params:
```bash
ros2 run pdm_test straight_driver --ros-args -p linear_speed:=0.3 -p angular_speed:=0.8 -p duration:=30.0
```

---

## Adding New Algorithm / Package

All algorithms live in `src/pdm_test/pdm_test/` as Python modules.

### 1. Create a new node (e.g., `my_planner.py`)
```bash
cat > src/pdm_test/pdm_test/my_planner.py << 'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyPlanner(Node):
    def __init__(self):
        super().__init__('my_planner')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Your algorithm here
        msg = Twist()
        msg.linear.x = 0.1
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 2. Add entry point to `setup.py`
```python
entry_points={
    'console_scripts': [
        'my_planner = pdm_test.my_planner:main',
    ],
},
```

### 3. Rebuild and run
```bash
colcon build --packages-select pdm_test --symlink-install
source install/setup.bash
ros2 run pdm_test my_planner
```

### 4. Add to launch file (optional)
Edit `src/pdm_test/launch/cafe.launch.py` and add a `Node` action to launch your algorithm automatically.

---

## Creating New Worlds

### Add a `.world` file
```bash
# Create your Gazebo world (e.g., my_world.world)
cp my_world.world src/pdm_test/worlds/
```

### Make TIAGo see it (required once per world)
```bash
cp src/pdm_test/worlds/my_world.world ~/tiago_public_ws/src/pal_gazebo_worlds/worlds/
cd ~/tiago_public_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch with your world
```bash
ros2 launch pdm_test cafe.launch.py world_name:=my_world
```

---

## Workspace Structure

```
src/pdm_test/
├── launch/
│   └── cafe.launch.py
├── pdm_test/
│   ├── __init__.py
│   ├── straight_driver.py
│   └── rrc_server.py
├── worlds/
│   ├── cafe.world
│   ├── cafe_table.world
│   ├── cafe_dynamic.world
│   └── empty.world
├── package.xml
├── setup.py
└── setup.cfg
```

**Note:** `build/`, `install/`, and `log/` are ignored and only exist locally.

---

 SLAM Mapping and Navigation

 Launch simulation with SLAM enabled
 Start TIAGo in Gazebo with SLAM and the navigation stack enabled:

 ros2 launch tiago_gazebo tiago_gazebo.launch.py \
   is_public_sim:=True \
   world_name:=walls \
   navigation:=True \
   slam:=True

 This launches:
 - Gazebo simulation
 - RViz visualisation
 - SLAM for online map building
 - Nav2 for planning (can be used after mapping)

 --------------------------------------------------

 Drive the robot (keyboard teleoperation)
 In a separate terminal, run:

 ros2 run teleop_twist_keyboard teleop_twist_keyboard

 Use the keyboard controls shown in the terminal to move the robot around the
 environment while SLAM builds the map.

 --------------------------------------------------

 Save the generated map
 When the map looks complete in RViz, save it using:

 ros2 run nav2_map_server map_saver_cli -f my_map

 This creates:
 - my_map.yaml
 - my_map.pgm

 in the current directory.

 --------------------------------------------------

 Use the saved map for navigation (no SLAM)
 To run navigation on a previously saved map, relaunch the simulation with SLAM
 disabled and the map loaded:

 ros2 launch tiago_gazebo tiago_gazebo.launch.py \
   is_public_sim:=True \
   world_name:=walls \
   navigation:=True \
   slam:=False \
   map:=/path/to/my_map.yaml

 You can now:
 - Send navigation goals in RViz
 - Or continue manual control using keyboard teleop

## Troubleshooting

| Issue | Solution |
|-------|----------|
| TIAGo spawns into empty world | Ensure `.world` file is copied to `~/tiago_public_ws/src/pal_gazebo_worlds/worlds/` and that workspace is rebuilt. |
| `ros2 launch` fails (package not found) | Ensure all three workspaces are sourced in order: ROS 2 → TIAGo → PDM. |
| Node not found | Run `colcon build` and `source install/setup.bash` in this workspace. |
