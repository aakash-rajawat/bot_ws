# Uncertainty-Aware SLAM Workspace

## 1. Overview

This repository is a ROS 2 Jazzy workspace for a simulated differential-drive robot with an uncertainty-aware SLAM pipeline. The system runs in Gazebo, builds uncertain wheel, LiDAR, and visual measurements, estimates relative motion with CUDA-accelerated DUGMA registration, and fuses timestamped relative-pose factors in a GTSAM pose graph.

<p align="center">
  <video
    src="https://github.com/aakash-rajawat/bot_ws/raw/main/demo_files/rviz_trimmed_cropped.mp4"
    controls
    muted
    loop
    width="100%">
  </video>
</p>

The main integrated launch path is:

```bash
ros2 launch bot_bringup vision_container.launch.py
```

That launch starts the headless maze simulation, joystick/control stack, uncertainty-aware sensing nodes, DUGMA relative-pose action servers and clients, GTSAM fusion, and odometry error evaluation.

## 2. Pipeline

The runtime pipeline is organized around relative-pose measurements with explicit covariance:

```text
Gazebo robot
  -> joint_states -> ua_wheel_odom -> /bot_controller/relative_pose_wheel
  -> /scan -> ua_lidar_point_cloud -> /ua_lidar/ua_point_cloud
  -> stereo images -> xfeat_lightglue_server -> vision_frontend -> /ua_triangulation/pointswithcovariance

uncertain point clouds
  -> mle_relative_pose_client
  -> MLERelativePose action server
  -> bot_dugma registration
  -> /bot_controller/relative_pose_mle_lidar
  -> /bot_controller/relative_pose_mle_triangulation

relative-pose factors
  -> gtsam_pose_slam
  -> /bot_controller/odom_gtsam_fused
  -> odom_error against Gazebo ground truth
```

Wheel odometry creates the primary timestamped graph timeline. LiDAR and triangulation relative-pose factors attach to nearby existing graph keys and use robust noise models in the GTSAM fusion node.

## 3. Workspace Layout

Key packages and directories:

- `src/bot_bringup`: integrated launch files for simulation, perception, registration, fusion, and evaluation.
- `src/bot_description`: robot URDF/Xacro, Gazebo sensor plugins, meshes, and RViz configs.
- `src/bot_worlds`: Gazebo maze and empty worlds plus textures and spawn configs.
- `src/bot_controller`: differential-drive control utilities and uncertainty-aware wheel odometry.
- `src/bot_vision_py`: Python XFeat/LightGlue correspondence service.
- `src/bot_vision`: C++ vision frontend, camera calibration handling, distortion covariance propagation, and triangulation.
- `src/bot_multisensor_odometry`: uncertainty-aware LiDAR point clouds, DUGMA action clients/servers, and relative-pose odometry integration.
- `src/bot_dugma`: CUDA-accelerated DUGMA registration library with ROPTLIB-based M-step optimization.
- `src/bot_localization`: GTSAM pose-graph fusion and optional EKF/AMCL launch files.
- `src/bot_mapping`: SLAM Toolbox mapping launch and config.
- `src/bot_navigation`: Nav2 planner/controller/behavior-tree configs.
- `src/bot_evaluation`: odometry error publisher against Gazebo ground truth.
- `notebooks`: derivation notebooks for uncertainty models.
- `scripts`: workspace setup, third-party fetch, and Python virtual environment bootstrap scripts.

## 4. Requirements

The workspace is designed around the provided container setup:

- Ubuntu 24.04 base image with ROS 2 Jazzy.
- CUDA-capable NVIDIA runtime for `bot_dugma` and the XFeat/LightGlue Python environment.
- `colcon`, `rosdep`, `vcstool`, CMake, and the ROS/Gazebo/Nav2/GTSAM dependencies resolved by `rosdep`.
- Third-party source checkouts from `third_party.repos`:
  - `ROPTLIB`
  - `accelerated_features`
- A separate workspace-local Python virtual environment at `xfeat_lightglue`, created by `scripts/bootstrap_workspace_venvs.sh`.

The devcontainer mounts `/dev/input`, uses host networking, and requests GPU access. Its `postCreateCommand` runs the first-run setup script.

## 5. Setup And Build

Inside the devcontainer, the first-run setup is:

```bash
bash scripts/first_run_workspace_setup.sh
```

The script registers the local rosdep override, fetches third-party repositories, verifies the XFeat/LightGlue weights, and creates the `xfeat_lightglue` virtual environment.

For a manual build after setup:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --merge-install
source install/setup.bash
```

The runtime Docker target also performs third-party fetch, `colcon build --merge-install`, and virtual environment bootstrap during image creation.

## 6. Run

Run the integrated uncertainty-aware SLAM pipeline:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bot_bringup vision_container.launch.py
```

Open RViz from a host-side process:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bot_bringup vision_host.launch.py
```

Other useful launch paths:

```bash
ros2 launch bot_description headless_gazebo.launch.py
ros2 launch bot_mapping slam.launch.py
ros2 launch bot_navigation navigation.launch.py
ros2 launch bot_controller joystick_teleop.launch.py
```

Important runtime topics and interfaces:

- `/bot_controller/relative_pose_wheel`
- `/ua_lidar/ua_point_cloud`
- `/ua_triangulation/pointswithcovariance`
- `/mle_relative_pose_lidar`
- `/mle_relative_pose_triangulation`
- `/bot_controller/relative_pose_mle_lidar`
- `/bot_controller/relative_pose_mle_triangulation`
- `/bot_controller/odom_gtsam_fused`
- `/bot_evaluation/odom_error`
- `/xfeat_lightglue`

## 7. Generated Code And Caveats

Several checked-in generated files are part of the normal build:

- `src/bot_controller/src/generated`: symbolic wheel-odometry covariance precomputes.
- `src/bot_vision/generated`: SymForce camera distortion covariance code and CasADi triangulation covariance code.
- `src/bot_multisensor_odometry/generated`: CasADi LiDAR point covariance code.
- `src/bot_dugma/src/generated`: DUGMA M-step polynomial basis, coefficient tiles, and quaternion-gradient helpers.

Generation scripts live next to the generated artifacts:

- `src/bot_dugma/m_step_opt/*.py`
- `src/bot_vision/symforce/*.py`
- `src/bot_vision/casadi/*.py`
- `src/bot_multisensor_odometry/casadi/*.py`

Known caveats in the current tree:

- `bot_bringup/vision_container.launch.py` is the canonical integrated launch path.
- `bot_localization/launch/global_localization.launch.py` currently has a misspelled launch entry function and should be treated as unfinished.
- Package metadata still contains TODO description fields in several packages.
- Some DUGMA and MLE client/server diagnostics are intentionally verbose while the pipeline is being tuned.
- `bot_vision_py/launch/xfeat_lightglue_server.launch.py` assumes the workspace path `/workspaces/bot_ws`.

## License

Original source code in this repository is licensed under the Apache License, Version 2.0. See `LICENSE`.

Third-party software, generated-code templates, model weights, and research publications retain their own licenses and attribution requirements. See `THIRD_PARTY_NOTICES.md` and `CITATION.cff`.

DUGMA and ROPTLIB are treated as research/external dependency references with unresolved upstream source-code licensing in this workspace audit. Do not copy or redistribute upstream DUGMA or ROPTLIB source code without explicit permission or license clarification.

## References

This workspace implements or builds on methods and software from:

- Henry, S., & Christian, J. A. (2024). LOSTU: Fast, Scalable, and Uncertainty-Aware Triangulation. https://doi.org/10.48550/arXiv.2311.11171
- Pu, C., Li, N., Tylecek, R., & Fisher, R. B. (2018). DUGMA: Dynamic Uncertainty-Based Gaussian Mixture Alignment. https://doi.org/10.48550/arXiv.1803.07426
- Dellaert, F., & GTSAM Contributors. (2022). borglab/gtsam (Version 4.2a8) [Computer software]. https://doi.org/10.5281/zenodo.5794541
- Potje, G., Cadar, F., Araujo, A., Martins, R., & Nascimento, E. R. (2024). XFeat: Accelerated Features for Lightweight Image Matching. https://doi.org/10.1109/CVPR52733.2024.00259
- Lindenberger, P., Sarlin, P.-E., & Pollefeys, M. (2023). LightGlue: Local Feature Matching at Light Speed. https://doi.org/10.48550/arXiv.2306.13643
- Carvalho Filho, J. G. N. D., Carvalho, E. A. N., Molina, L., & Freire, E. O. (2019). The Impact of Parametric Uncertainties on Mobile Robots Velocities and Pose Estimation. IEEE Access, 7, 69070-69086. https://doi.org/10.1109/ACCESS.2019.2919335
