# Planning-and-decision-making-RO47005  
**Mobile manipulator motion planning and decision making.**

This workspace contains the ROS 2 package **`pdm_test`**, which provides custom Gazebo worlds and a launch file to run the **TIAGo** robot from PAL Robotics in a variety of restaurant/café environments.

The included worlds are:

- `cafe.world`
- `cafe_table.world`
- `cade_dynamic.world`

You may add more worlds; instructions are included at the bottom of this README.

---

# 1. Requirements

Before using this workspace, the machine must already have:

1. **ROS 2 Humble**
2. **PAL Robotics' public TIAGo simulation workspace**  
   cloned and built here:

   ```
   ~/tiago_public_ws
   ```

This repository acts as an **overlay workspace** on top of `tiago_public_ws`.

---

# 2. Cloning this workspace

clone the repository:

```bash
cd ~
git clone git@github.com:raaftw/PDM_tiago_ws.git
```


# 3. Install dependencies (first-time setup)

Make sure the TIAGo simulation workspace is sourced:

```bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
```

Then build this overlay workspace:

```bash
cd ~/PDM_tiago_ws
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

You will need to source these three lines in any new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source ~/PDM_tiago_ws/install/setup.bash
```

---

# 4. Copy custom worlds into TIAGo’s world directory (required once)

PAL’s TIAGo simulation loads worlds using a **name** (`world_name:=…`), not a file path.  
Therefore, the `.world` files must also exist in the TIAGo world directory:

```bash
cd ~/tiago_public_ws/src/pal_gazebo_worlds/worlds

cp ~/PDM_tiago_ws/src/pdm_test/worlds/cafe.world .
cp ~/PDM_tiago_ws/src/pdm_test/worlds/cafe_table.world .
cp ~/PDM_tiago_ws/src/pdm_test/worlds/cade_dynamic.world .
```

Rebuild TIAGo’s workspace:

```bash
cd ~/tiago_public_ws
colcon build --symlink-install
source install/setup.bash
```

Now TIAGo can load:

```
world_name:=cafe
world_name:=cafe_table
world_name:=cade_dynamic
```

---

# 5. Running the simulation

Use the launch file in `pdm_test`:

### Default world (cafe)

```bash
ros2 launch pdm_test cafe.launch.py
```

### Other custom worlds

```bash
ros2 launch pdm_test cafe.launch.py world_name:=cafe_table
```

```bash
ros2 launch pdm_test cafe.launch.py world_name:=cade_dynamic
```

### TIAGo’s default world

```bash
ros2 launch pdm_test cafe.launch.py world_name:=pal_office
```

---

# 6. Workspace structure

```
PDM_tiago_ws/
├── .gitignore
├── README.md
└── src/
    └── pdm_test/
        ├── launch/
        │   └── cafe.launch.py
        ├── worlds/
        │   ├── cafe.world
        │   ├── cafe_table.world
        │   └── cade_dynamic.world
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── pdm_test/
            └── __init__.py
```

**NOTE:**  
The following directories are *local build artefacts* and correctly ignored by `.gitignore`:

- `build/`
- `install/`
- `log/`

These must not be pushed to GitHub.

---

# 7. Adding new worlds

To add a new world (e.g. `cafe.world`):

### 1. Add it to this workspace:

```bash
cp cafe.world ~/PDM_tiago_ws/src/pdm_test/worlds/
```

### 2. Also add it to TIAGo’s world directory:

```bash
cp ~/PDM_tiago_ws/src/pdm_test/worlds/cafe.world \
   ~/tiago_public_ws/src/pal_gazebo_worlds/worlds/
```

### 3. Rebuild TIAGo’s workspace:

```bash
cd ~/tiago_public_ws
colcon build --symlink-install
```

### 4. Launch:

```bash
ros2 launch pdm_test cafe.launch.py world_name:=cafe
```

As long as the file is named:

```
cafe.world
```

it can be launched via:

```
world_name:=cafe
```

---

# 8. Troubleshooting

### TIAGo spawns into an empty world
Most likely cause:

- The `.world` file was not copied into  
  `~/tiago_public_ws/src/pal_gazebo_worlds/worlds`

### `ros2 launch pdm_test cafe.launch.py` cannot find TIAGo launch files
Make sure you sourced the TIAGo workspace:

```bash
source ~/tiago_public_ws/install/setup.bash
```