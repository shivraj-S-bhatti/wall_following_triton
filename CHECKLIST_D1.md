# Deliverable 1 Execution Checklist

## 1) VM Setup (One Time)

- [ ] Clone or pull this repository in `~/catkin_ws/src`
- [ ] Install Triton simulator package (`stingray_sim`)
- [ ] Build workspace (`catkin_make`)
- [ ] Source setup (`source ~/catkin_ws/devel/setup.bash`)

## 2) Bringup and Smoke Test

- [ ] Run: `roslaunch wall_following_triton wf_d1_demo.launch`
- [ ] Confirm Gazebo starts and `wf_policy_node` starts cleanly
- [ ] Verify command output: `rostopic hz /cmd_vel` (target >= 10 Hz)
- [ ] Verify no crash for at least 60 seconds

## 3) Straight-Wall Validation (D1 scope)

- [ ] Run 5 starts along straight walls
- [ ] Robot keeps moving while following right wall
- [ ] Front obstacle triggers clear left recovery
- [ ] No unhandled exceptions across all runs

## 4) Demo Video

- [ ] Record one short demo showing successful straight-wall following
- [ ] Compress video under 20 MB (if needed)
- [ ] Confirm playback quality clearly shows behavior

## 5) Submission Packaging

- [ ] Confirm package contains scripts, launch, config, `package.xml`, `CMakeLists.txt`
- [ ] Create archive: `P2D1_firstname_lastname.tar` or `.tar.gz`
- [ ] Include exactly two submission items:
  - [ ] ROS package
  - [ ] Video file or public video link reference
- [ ] Upload to Canvas portal `P2-D1`

## 6) Final Pre-Submit Audit

- [ ] `package.xml` maintainer fields updated
- [ ] Launch command in README works as documented
- [ ] Archive opens correctly and contains expected files
