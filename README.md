# Planning-and-decision-making-RO47005
Mobile manipulator motion planning and decision making.

# TIAGo Simulation Environment — Custom Cafe Worlds

This repository contains a ROS 2 package **`pdm_test`** that provides multiple Gazebo worlds for use with **PAL Robotics’ TIAGo public simulation**.  
It allows launching TIAGo inside different environments using a simple launch argument.

Included worlds:

- `cafe.world`
- `cafe_table.world`
- `cade_dynamic.world`

You can add more worlds easily (see the bottom section).

---

# 1. Prerequisites

Before using this package, you need:

- **ROS 2 Humble** installed
- PAL Robotics’ **TIAGo public simulation workspace**  
  (referred to as: `tiago_public_ws`)  
  This provides `tiago_gazebo` and all associated robot simulation packages.

This README explains how to integrate **this package** with that simulation.

---

# 2. Clone this repository (assignment workspace)

The workspace for this assignment is assumed to be:

```
~/PDM_tiago_ws
```

Clone your repo into the `src/` subdirectory:

```bash
mkdir -p ~/PDM_tiago_ws/src
cd ~/PDM_tiago_ws/src
git clone https://github.com/YOUR_USERNAME/Planning-and-decision-making-RO47005.git
```

Make sure the folder structure looks like:

```
~/PDM_tiago_ws/src/pdm_test
```

---

# 3. Build the overlay workspace

From inside your assignment workspace:

```bash
cd ~/PDM_tiago_ws
colcon build --symlink-install
```

Every time you open a new terminal, source the workspaces **in this order**:

```bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source ~/PDM_tiago_ws/install/setup.bash
```

(Add the last two lines to `~/.bashrc` if you want them always active.)

---

# 4. Copy the custom worlds into TIAGo’s simulation (required once)

PAL’s simulator loads worlds using a **name** (world_name:=…), not file paths.  
Therefore, your custom worlds must also exist inside the TIAGo simulation folder:

```bash
cd ~/tiago_public_ws/src/pal_gazebo_worlds/worlds

cp ~/PDM_tiago_ws/src/pdm_test/worlds/cafe.world .
cp ~/PDM_tiago_ws/src/pdm_test/worlds/cafe_table.world .
cp ~/PDM_tiago_ws/src/pdm_test/worlds/cade_dynamic.world .
```

Then rebuild TIAGo’s workspace once:

```bash
cd ~/tiago_public_ws
colcon build --symlink-install
source install/setup.bash
```

Now TIAGo will recognise:

```
world_name:=cafe
world_name:=cafe_table
world_name:=cade_dynamic
```

---

# 5. Running the simulation

Launch TIAGo using the `pdm_test` launch file.

### Default world (cafe.world)

```bash
ros2 launch pdm_test cafe.launch.py
```

### Other worlds

```bash
ros2 launch pdm_test cafe.launch.py world_name:=cafe_table
```

```bash
ros2 launch pdm_test cafe.launch.py world_name:=cade_dynamic
```

### PAL default world (pal_office)

```bash
ros2 launch pdm_test cafe.launch.py world_name:=pal_office
```

Internally, this launch file:

- Adds `pdm_test/worlds` to the `GAZEBO_RESOURCE_PATH`
- Calls PAL’s `tiago_gazebo.launch.py`
- Passes the selected world via `world_name:=...`
- Ensures public simulation mode is enabled

---

# 6. Directory overview (where to work)

Most edits for this assignment happen here:

```
~/PDM_tiago_ws/src/pdm_test
```

Build your workspace here:

```
~/PDM_tiago_ws
```

TIAGo simulation code (DO NOT modify unless needed):

```
~/tiago_public_ws
```

---

# 7. Adding new worlds

To add a new world (e.g. `new_scene.world`), follow these steps:

### 1 — Add to your package

Place the `.world` file into:

```
~/PDM_tiago_ws/src/pdm_test/worlds/
```

Example:

```bash
cp new_scene.world ~/PDM_tiago_ws/src/pdm_test/worlds/
```

### 2 — Copy into TIAGo’s world directory

Required so that TIAGo can load it via `world_name:=new_scene`:

```bash
cp ~/PDM_tiago_ws/src/pdm_test/worlds/new_scene.world \
   ~/tiago_public_ws/src/pal_gazebo_worlds/worlds/
```

### 3 — Rebuild TIAGo’s workspace once

```bash
cd ~/tiago_public_ws
colcon build --symlink-install
```

### 4 — Launch it

```bash
ros2 launch pdm_test cafe.launch.py world_name:=new_scene
```

As long as the file is named:

```
new_scene.world
```

you may run:

```
world_name:=new_scene
```

---

# 8. Notes

- Worlds kept in this Git repo ensure reproducibility.
- Worlds copied into TIAGo’s simulation package enable TIAGo’s selector (`world_name:=...`).
- This README assumes ROS 2 Humble and the public TIAGo simulation from PAL Robotics.

---

If you need a developer-focused README (navigation, MoveIt, mapping, planning), ask and it can be added here.
