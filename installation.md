# Autonomous UAV Navigation System — Full Setup Guide (Fresh Install)

This document covers everything needed to go from a fresh Ubuntu 24.04 install
to a working PX4 + ROS 2 Jazzy + Gazebo Harmonic autonomous drone simulation,
including every fix discovered during setup. Follow top to bottom in order.

---

## 0. System Requirements

- Ubuntu 24.04
- Python 3.10+ (ships with 24.04 by default)
- At least 20 GB free disk space (PX4 + Gazebo builds are large)

---

## 1. Base Dependencies

```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg git wget
```

---

## 2. Install Gazebo Harmonic

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y gz-harmonic
```

---

## 3. Install ROS 2 Jazzy

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Initialize rosdep (required once, easy to forget):**
```bash
sudo rosdep init
rosdep update
```

---

## 4. Install PX4 Firmware (SITL)

```bash
mkdir -p ~/px4_ros2_ws
cd ~/px4_ros2_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

> If this script tries to install a different Gazebo version and conflicts
> with Harmonic, skip/decline the Gazebo-specific prompts — Harmonic is
> already installed from Step 2.

Build the SITL target once to confirm it compiles:
```bash
make px4_sitl
```

---

## 5. Install Micro XRCE-DDS Agent (PX4 ⟷ ROS 2 bridge)

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

---

## 6. Install PX4 ROS 2 Message Definitions

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

---

## 7. Python Dependencies

```bash
sudo apt install -y python3-numpy python3-scipy python3-opencv
pip3 install transforms3d --break-system-packages
```

> Ubuntu 24.04 blocks unmanaged `pip install` by default (PEP 668).
> `--break-system-packages` is safe here and standard practice for ROS 2 dev
> machines — prefer `apt` packages first, `pip` only when apt doesn't have it.

---

## 8. Clone This Project

```bash
cd ~/ros2_ws/src
git clone Private REPO
```

---

## 9. Install QGroundControl (required — PX4 refuses to arm without a GCS link)

```bash
cd ~
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
wget https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
```

---

## 10. CRITICAL FIXES — apply before your first launch

These four fixes are required. Without them, the drone will spawn but never
arm or fly correctly. They only need to be done once per machine/checkout.

### 10a. Copy the custom world files into PX4's world directory

PX4 only looks for world files inside its own `Tools/simulation/gz/worlds/`
folder — the repo's own `Worlds/` folder is never referenced automatically.

```bash
cp ~/ros2_ws/src/Autonomous-UAV-Navigation-System/Worlds/*.sdf \
   ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 10b. Fix the internal `<world name>` tag in each world file

Both `urban.sdf` and `corridor.sdf` ship with `<world name="default">`
internally, which doesn't match their filename. PX4 listens for Gazebo
topics named after the `PX4_GZ_WORLD` env var, so this mismatch causes PX4
to hang forever on `Waiting for Gazebo world...`.

```bash
sed -i 's/<world name="default">/<world name="urban">/' \
    ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/urban.sdf

sed -i 's/<world name="default">/<world name="corridor">/' \
    ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/corridor.sdf
```

### 10c. Add the missing magnetometer plugin to both world files

Neither world loads the Gazebo magnetometer system plugin, which causes a
permanent `Preflight Fail: Found 0 compass` arming block.

```bash
sed -i '/gz-sim-navsat-system/a\    <plugin name="gz::sim::systems::Magnetometer" filename="gz-sim-magnetometer-system"/>' \
    ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/urban.sdf

sed -i '/gz-sim-navsat-system/a\    <plugin name="gz::sim::systems::Magnetometer" filename="gz-sim-magnetometer-system"/>' \
    ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/corridor.sdf
```

**Verify both fixes landed:**
```bash
grep -A1 "world name\|magnetometer" ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/urban.sdf
```

### 10d. Fix PX4 topic-versioning mismatch in the repo's Python nodes

Newer PX4 builds (v1.18 beta and later) publish some topics under versioned
names (`_v1`, `_v4`, etc). The repo's code was written against the old
unversioned names, so several ROS 2 nodes silently receive nothing — this is
the root cause behind "armed but never actually flies."

```bash
cd ~/ros2_ws/src/Autonomous-UAV-Navigation-System/autonomous_drone

sed -i 's#"/fmu/out/vehicle_local_position"#"/fmu/out/vehicle_local_position_v1"#g' \
    depth_avoidance.py mapping_node.py path_planner.py

sed -i 's#"/fmu/out/vehicle_status"#"/fmu/out/vehicle_status_v4"#g' \
    depth_avoidance.py
```

**Verify:**
```bash
grep -n '"/fmu/out/' depth_avoidance.py mapping_node.py path_planner.py
```

> ⚠️ **If a future/older PX4 checkout is used**, run
> `ros2 topic list | grep fmu` after first launch to check whether these
> topic names still apply. If PX4's version changes, the versioned suffixes
> may need to be updated again — check `ros2 topic info <topic> --verbose`
> and look for `Publisher count: 0` on the unversioned name as the tell.

### 10e. (Optional) Lower flight altitude

Default cruise altitude is 5.0 m, which can look too high above small maps
like `corridor.sdf`. To change it:

```bash
sed -i 's/("flight_altitude", 5.0)/("flight_altitude", 2.0)/' \
    ~/ros2_ws/src/Autonomous-UAV-Navigation-System/autonomous_drone/depth_avoidance.py
```

---

### 10f. Fix(Complusory) — install the bridge package

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
```

## 11. Build the Workspace

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 12. Fix the RViz Display Config (one-time, do it during first launch)

The shipped `mapping.rviz` only loads Global Options — the actual map/marker
displays don't render out of the box. After your first launch (see Step 13),
in the RViz window:

1. Click **Add** (bottom of Displays panel) → **By Topic** tab.
2. Add:
   - `/map` → **Map**
   - `/map_3d` → **MarkerArray**
   - `/drone_marker` → **Marker**
3. Confirm **Fixed Frame** (top of Global Options) is set to `map`.
4. `File → Save Config As` → overwrite `mapping.rviz` so this persists.

---

## 13. Launch Sequence (every time you run the project)

**Terminal 1 — main stack (PX4 + Gazebo + ROS nodes + RViz):**
```bash
pkill -9 -f px4; pkill -9 -f MicroXRCEAgent; pkill -9 -f gz   # clear any stale processes

export PX4_GZ_WORLD=urban     # or: corridor
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch autonomous_drone drone.launch.py
```

**Terminal 2 — QGroundControl (passive, required for arming):**
```bash
cd ~
./QGroundControl-x86_64.AppImage
```
> Leave it open and connected. **Never touch its Arm/Takeoff/Fly controls** —
> it exists purely to satisfy PX4's "GCS connected" health check. Touching
> its flight controls conflicts with the ROS 2 node's own control and can
> cause an immediate disarm.

**That's it — no 3rd manual node needed.** `depth_avoidance.py` (launched
automatically as part of Terminal 1) handles arming, Offboard mode switching,
takeoff, and navigation entirely on its own.

### What should happen automatically:
1. PX4 boots, connects to QGC, compass/GPS checks pass.
2. `depth_avoidance` self-arms and climbs to `flight_altitude` within ~10–15s
   — watch for these two log lines in Terminal 1:
   ```
   OFFBOARD mode
   ARMED - Takeoff
   ```
3. Once hovering, click **"2D Goal Pose"** in RViz's toolbar (NOT "2D Pose
   Estimate" — that's a different tool that does nothing here), then
   click-and-drag on the map to set a destination + heading.
4. Watch for `Goal: (x, y)` in the log, then the drone should fly toward it
   while `mapping_node` populates the voxel occupancy grid in RViz.

---

## 14. One-Time PX4 Parameter Tweak (do this the first time you arm)

PX4's `COM_DISARM_PRFLT` safety timer can auto-disarm the vehicle if it
doesn't detect a real climb within ~10 seconds — which can trigger falsely
under a loaded SITL simulation (Gazebo + RViz + mapping all running at once).
Disable it via QGroundControl's **MAVLink Console** (hamburger menu →
Analyze Tools → MAVLink Console):

```
param set COM_DISARM_PRFLT -1
param save
```

This only needs to be done once — `param save` persists it in PX4's stored
parameters for that build.

---

## 15. Full Kill / Clean Reset

Run this any time things get stuck, before every fresh relaunch, or if a
previous session didn't shut down cleanly:

```bash
pkill -9 -f px4
pkill -9 -f MicroXRCEAgent
pkill -9 -f gz
pkill -9 -f ruby          # Gazebo GUI sometimes spawns this too
pkill -9 -f rviz2         # optional — kill RViz if it's hung
```

**Check nothing is still holding the agent's port before relaunching:**
```bash
sudo lsof -i :8888
```
If this prints anything, kill that specific PID:
```bash
sudo kill -9 <PID>
```

**One-liner version** (safe to run before every launch, does nothing if
nothing's running):
```bash
pkill -9 -f px4; pkill -9 -f MicroXRCEAgent; pkill -9 -f gz; pkill -9 -f ruby
```

---

## 16. Tunable Parameters — What They Do and Where to Change Them

All of these live in `depth_avoidance.py` and `path_planner.py` inside
`~/ros2_ws/src/Autonomous-UAV-Navigation-System/autonomous_drone/`, unless
noted otherwise. After editing any of these, rebuild:
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select autonomous_drone && source install/setup.bash
```

| Parameter | Default | What it controls | Change if... |
|---|---|---|---|
| `flight_altitude` | `5.0` m | Cruise altitude the drone climbs to and flies at | Map feels too small for 5m — lower to `2.0`–`3.0` for `corridor.sdf` |
| `safety_radius` | `0.6` m | Clearance buffer A* keeps around obstacles | Drone clips buildings → increase; drone can't fit through tight gaps → decrease |
| `base_speed` | `5.0`–`7.5` m/s | Forward cruising speed | Want a slower, easier-to-follow demo → lower to `2.0`–`3.0` |
| `front_stop_threshold` | `1.2` m | Distance at which it emergency-stops for an obstacle ahead | Stopping too early/late relative to how tight your corridor is |
| `replan_rate` | `2.0` Hz | How often the global A* path recalculates | Higher = more responsive to new obstacles, more CPU load |
| `map_resolution` | `0.10` m | Voxel size of the occupancy grid | Lower = finer detail but heavier on CPU/RViz; raise if RViz is laggy |

### How to change `flight_altitude` (worked example)

```bash
grep -n "flight_altitude" ~/ros2_ws/src/Autonomous-UAV-Navigation-System/autonomous_drone/depth_avoidance.py
```
Find the line:
```python
self.declare_parameters("", [("flight_altitude", 5.0)])
```
Change the value directly:
```bash
sed -i 's/("flight_altitude", 5.0)/("flight_altitude", 2.0)/' \
    ~/ros2_ws/src/Autonomous-UAV-Navigation-System/autonomous_drone/depth_avoidance.py
```
Rebuild and relaunch as shown above — this value is read once at node
startup, so a running instance won't pick up the change without a restart.

> Note: these are declared as ROS 2 parameters, which technically means you
> *could* override them at launch time without editing source, e.g.
> `ros2 launch autonomous_drone drone.launch.py flight_altitude:=2.0` —
> **but only if `drone.launch.py` actually exposes them as launch
> arguments**. Check with `grep DeclareLaunchArgument launch/drone.launch.py`
> first; if they're not exposed there, editing the Python default (as above)
> is the only way.

---

## 17. RViz Toolbar — What "2D Pose Estimate" vs "2D Goal Pose" Actually Do

These two buttons look similar and sit right next to each other in RViz's
toolbar — easy to click the wrong one.

- **2D Pose Estimate** — tells a localization system "the robot is currently
  *here*, facing this direction." It's used to manually correct/seed a
  robot's belief about its own position (e.g. for AMCL-style localization on
  real robots). **This project doesn't use it at all** — clicking it does
  nothing for navigation, since `depth_avoidance.py` already gets ground
  truth position directly from PX4's `vehicle_local_position` topic.

- **2D Goal Pose** — publishes a `PoseStamped` message to `/goal_pose`, which
  is exactly what `depth_avoidance.py` is waiting on to start flying. This is
  the **only** button in RViz that actually triggers movement in this
  project. Click-and-drag (not just a single click) sets both the target
  location and the heading the drone should approach it from.

**Rule of thumb for this project: ignore "2D Pose Estimate" entirely, only
ever use "2D Goal Pose" to send the drone somewhere.**

---

## 18. Fix Patchy/Front-Only LiDAR Scans — Merge the Repo's 360° LiDAR into PX4's Model

### Why this happens

`drone.launch.py` spawns PX4's **stock `x500_depth` model** (via
`make px4_sitl gz_x500_depth`), which only has a narrow-FOV OakD-Lite stereo
depth camera — no spinning LiDAR. Meanwhile the repo ships its own
`drone_model/model.sdf` with a genuine 360° `gpu_lidar` sensor, but that
custom model is **never actually spawned** — it just sits unused in the repo.

This mismatch is also why `static_transform_publisher` broadcasts a TF to a
frame called `x500_depth_0/lidar_link_custom/lidar_custom` (visible in the
launch logs) that doesn't correspond to any real sensor in the running
model — the whole pipeline was clearly designed assuming this LiDAR link
would be merged in, but the merge step was missing.

**Result:** `/scan` data is patchy/front-only instead of a full sweep, and
the resulting occupancy map looks messier than the README's demo video.

### The fix — transplant the LiDAR sensor into PX4's local x500_depth model

**Step 1 — back up PX4's stock model first:**
```bash
cp ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf \
   ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf.bak
```

**Step 2 — insert the LiDAR link + sensor + joint:**
```bash
nano ~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf
```

Find the closing `</model>` tag at the very end of the file, and paste this
**right before it**:

```xml
    <link name="lidar_link_custom">
      <pose>0 0 0.35 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000166667</ixx>
          <iyy>0.000166667</iyy>
          <izz>0.000166667</izz>
        </inertia>
      </inertial>
      <visual name="lidar_visual_custom">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.1 0.1 0.1 1.0</diffuse>
          <ambient>0.1 0.1 0.1 1.0</ambient>
        </material>
      </visual>
      <collision name="lidar_collision_custom">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
      </collision>
      <sensor name="lidar_custom" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose>
        <topic>/scan</topic>
        <always_on>1</always_on>
        <visualize>true</visualize>
        <update_rate>20</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.20</min>
            <max>25.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <joint name="lidar_joint_custom" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link_custom</child>
    </joint>
```

Save and close (Ctrl+O, Enter, Ctrl+X).

**Step 3 — clean relaunch:**
```bash
pkill -9 -f px4; pkill -9 -f MicroXRCEAgent; pkill -9 -f gz

export PX4_GZ_WORLD=corridor
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch autonomous_drone drone.launch.py
```

**Verify it worked:**
```bash
ros2 topic echo /scan --once
```
You should see `ranges` with **360 values** now (a full sweep), not a narrow
cone. The map should fill in far more evenly and completely as the drone
flies, matching the README's demo instead of a front-only patchy result.

> ⚠️ Note: this edit lives inside PX4-Autopilot's own tree
> (`~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf`),
> **not** inside the ROS 2 workspace — so it does **not** get restored by
> `git pull` on the repo, but it also won't survive a fresh PX4-Autopilot
> re-clone. Keep this README handy to redo it if you ever re-clone PX4.

---

## 19. Free Camera Control in Gazebo (Fix "Camera Stuck to Drone")

### Why this happens

PX4's own SITL startup script actively issues a Gazebo "follow" command on
launch (visible in the logs as `Setting camera to follow x500_depth_0` and
`Camera follow offset set to ...`). This re-locks the camera to the drone
every frame, overriding manual orbit/pan/zoom attempts.

### Fix — release the follow lock with one command

While the sim is running, in any terminal:
```bash
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""'
```

The camera immediately unlocks. You can then freely:
- **Orbit** — left-click and drag
- **Pan** — right-click (or middle-click) and drag
- **Zoom** — scroll wheel

using the **"Move Camera"** tool in the Gazebo toolbar.

> Run this command again any time the camera re-locks (e.g. after a
> relaunch, since PX4's startup script re-issues the follow command every
> time).

---

## Troubleshooting Quick Reference

| Symptom | Cause | Fix |
|---|---|---|
| `Waiting for Gazebo world...` forever | World name tag mismatch | Step 10b |
| `Found 0 compass` | Missing magnetometer plugin | Step 10c |
| `No connection to the GCS` | QGroundControl not running | Step 13, Terminal 2 |
| Armed then `Disarmed by preflight inaction` | `COM_DISARM_PRFLT` timeout | Step 14 |
| Armed, motors spin, but never climbs | Position/status topics not reaching ROS 2 nodes (PX4 version mismatch) | Step 10d |
| RViz completely empty despite data flowing | Missing display configs | Step 12 |
| Drone doesn't move after "2D Goal Pose" click | Used "2D Pose Estimate" instead, or click wasn't a drag | Re-click using drag-and-release |
| `externally-managed-environment` on `pip install` | Ubuntu 24.04 PEP 668 | Use `--break-system-packages` or prefer `apt` |
| `rosdep: not initialized` | One-time init skipped | `sudo rosdep init && rosdep update` |
| LiDAR only sees front, map looks patchy/messy vs README demo | Spawned model has no real 360° LiDAR — TF points at a sensor that doesn't exist | Step 18 |
| Gazebo camera stuck following the drone, can't orbit/pan/zoom | PX4's startup script auto-issues a camera-follow lock | Step 19 |

---

## Quick Reference: All Key Paths

| What | Path |
|---|---|
| PX4 Firmware | `~/px4_ros2_ws/PX4-Autopilot` |
| PX4 world files | `~/px4_ros2_ws/PX4-Autopilot/Tools/simulation/gz/worlds/` |
| ROS 2 workspace | `~/ros2_ws` |
| This repo (source) | `~/ros2_ws/src/Autonomous-UAV-Navigation-System` |
| px4_msgs | `~/ros2_ws/src/px4_msgs` |
| QGroundControl | `~/QGroundControl-x86_64.AppImage` |
| PX4 flight logs (.ulg) | `~/px4_ros2_ws/PX4-Autopilot/build/px4_sitl_default/rootfs/log/` |
