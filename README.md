# CS603 Particle Filter

ROS Noetic package for COMPSCI 603 Project 3: robot localization with
odometry motion models, likelihood-field sensor models, and a particle filter.

This workspace now includes the full Deliverable 1 foundation:

- a reusable map utility layer
- an odometry-based motion model sampler
- a likelihood-field sensor model backed by a distance transform
- a Deliverable 1 runtime node that executes both models in Gazebo
- a standalone likelihood-field generation script for map and PNG outputs

## Package Layout

- `scripts/map_utils.py`: map loading, world/map conversions, distance field
- `scripts/motion_model.py`: odometry RTR motion model sampler
- `scripts/sensor_model.py`: likelihood-field LiDAR scoring
- `scripts/particle_filter.py`: Deliverable 1 runtime node and D2 foundation
- `scripts/generate_likelihood_field.py`: offline metric-map/field generation
- `scripts/position_publisher.py`: publishes `/odom` from Gazebo model state
- `scripts/teleop_particle_filter.py`: keyboard teleop using +x forward
- `config/particle_filter_params.yaml`: default model parameters
- `maps/README.md`: where to place the saved house map

## Dependencies

Install ROS and the project dependencies on Ubuntu 20.04:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  python3-catkin-tools \
  python3-numpy \
  python3-pil \
  python3-yaml \
  python3-pip

pip3 install --user pynput
```

The house world is provided by Turtlebot3 Gazebo:

```bash
cd ~/catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws
catkin_make
```

Clone this package into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://gitlab.com/HCRLab/stingray-robotics/cs603_particle_filter.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Deliverable 1 Workflow

### 1. Build the house map

Launch GMapping:

```bash
roslaunch cs603_particle_filter triton_gmapping.launch
```

On a second terminal, drive the robot:

```bash
rosrun cs603_particle_filter teleop_particle_filter.py
```

Use mapping-safe motion only: `W/S` or `Up/Down` for forward/back, and
`A/D`, `Left/Right`, or `Q/E` for in-place rotation. Lateral strafing is
disabled because it can skew the SLAM map and does not match the odometry
motion model used by the particle filter.

After the map is complete, save it:

```bash
rosrun map_server map_saver -f ~/house_map
```

Copy `~/house_map.yaml` and `~/house_map.pgm` into this package's `maps/`
directory, or pass the absolute YAML path through the launch file.

### 2. Generate the metric map and likelihood field

```bash
rosrun cs603_particle_filter generate_likelihood_field.py \
  --map-yaml $(rospack find cs603_particle_filter)/maps/house_map.yaml \
  --metric-map-output $(rospack find cs603_particle_filter)/artifacts/house_metric_map.png \
  --likelihood-field-output $(rospack find cs603_particle_filter)/artifacts/house_likelihood_field.png \
  --cache-output $(rospack find cs603_particle_filter)/artifacts/house_likelihood_field.npz
```

This produces:

- a submission-ready metric map PNG
- a submission-ready likelihood-field PNG
- a cached numeric distance field for runtime reuse

### 3. Run Deliverable 1

```bash
roslaunch cs603_particle_filter particle_filter.launch \
  map_yaml:=$(rospack find cs603_particle_filter)/maps/house_map.yaml
```

The launch file starts Gazebo, RViz, the robot pose publisher, and the
Deliverable 1 node. The node:

- subscribes to `/odom` and `/scan`
- samples motion-model predictions from consecutive odometry readings
- publishes motion uncertainty as a `PoseArray` on `/motion_model_samples`
- evaluates the live LiDAR scan against the likelihood field
- publishes the current pose and log-likelihood diagnostics

## Key Runtime Parameters

The defaults live in `config/particle_filter_params.yaml`.

- `map_yaml`: YAML file produced by `map_saver`
- `beam_step`: LiDAR beam subsampling factor
- `max_usable_range`: max range included in scoring
- `sigma_hit`: Gaussian sensor noise width
- `motion_alpha{1,2,3,4}`: odometry noise coefficients
- `motion_sample_count`: number of motion samples published for debug
- `metric_map_output_png`: output path for the generated metric map PNG
- `likelihood_field_output_png`: output path for the generated field PNG
- `likelihood_field_cache`: `.npz` cache for the numeric distance field

## Deliverable 2 Reuse

Deliverable 2 is implemented in `scripts/particle_filter.py`.
The core interfaces remain isolated:

- `OdometryMotionModel.sample_motion(pose, odom_prev, odom_curr) -> pose`
- `LikelihoodFieldSensorModel.scan_log_likelihood(pose, scan_msg) -> float`

The runtime node:

- initializes particles globally in free map cells
- predicts particles with odometry
- weights particles with LiDAR likelihoods
- estimates pose from weighted particles
- resamples with low-variance resampling
- publishes particles to `/particle_filter/particles`
- publishes the estimate to `/particle_filter/estimated_pose`
- logs D3 metrics to `artifacts/pf_run.csv`

Run the D2 demo:

```bash
roslaunch cs603_particle_filter particle_filter.launch \
  map_yaml:=$(rospack find cs603_particle_filter)/maps/house_map.yaml \
  csv_log_path:=$(rospack find cs603_particle_filter)/artifacts/pf_run.csv
```

In RViz, add displays for:

- `PoseArray`: `/particle_filter/particles`
- `PoseStamped`: `/particle_filter/estimated_pose`
- `PoseStamped`: `/particle_filter/current_pose`
- `Map`: `/map`

In another terminal, drive the robot:

```bash
rosrun cs603_particle_filter teleop_particle_filter.py
```

Record a demo video showing global initialization, scan weighting/convergence,
robot motion, and particle updates.

## Deliverable 3 Metrics

After the D2 demo run, generate the quantitative figure:

```bash
python3 $(rospack find cs603_particle_filter)/scripts/plot_pf_metrics.py \
  --csv $(rospack find cs603_particle_filter)/artifacts/pf_run.csv \
  --output $(rospack find cs603_particle_filter)/artifacts/pf_metrics.png
```

The D3 draft lives in `report_d3/main.tex`. Fill the TODOs after the experiment
screenshots and metrics are available.

## Submission Notes

Deliverable 1 should include:

- the ROS package
- `house_metric_map.png`
- `house_likelihood_field.png`

The package metadata and launch file are set up so the grader can run the demo
once the saved house map is included.
