# TA Setup (Ubuntu 20.04, ROS Noetic)

From the extracted `wall_following_triton` package directory:

```bash
chmod +x setup_d1.sh
./setup_d1.sh
```

Then run the demo:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch wall_following_triton wf_d1_demo.launch
```

## What setup_d1.sh does

- installs required apt packages:
  - `ros-noetic-desktop-full`
  - `ros-noetic-gazebo-ros-pkgs`
  - `ros-noetic-depthimage-to-laserscan`
  - `ros-noetic-gmapping`
  - `python3-catkin-tools`
  - `python3-pip`
  - `python3-yaml`
- installs `pynput` via `pip3 --user`
- clones/updates `stingray_sim` into `~/catkin_ws/src`
- links `wall_following_triton` into `~/catkin_ws/src`
- runs `catkin_make`
- verifies `stingray_sim` and `wall_following_triton` are discoverable by ROS
