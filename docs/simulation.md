# Run the robot simulation

## Dependencies

Use the [USI RoboMaster framework](https://github.com/idsia-robotics/robotics-lab-usi-robomaster)
to install ROS 2 Humble, `robomaster_ros`, the patched RoboMaster SDK, and the
CoppeliaSim RoboMaster plugin. Follow its platform-specific installation and build
instructions. This repository contains the navigation package and scene; it does
not bundle ROS, the SDK, or compiled plugins.

The tested configuration is macOS ARM64, CoppeliaSim 4.7, Python 3.10, ROS Humble,
OpenCV 4.6, and NumPy 1.26. Other platforms and fresh dependency installations have
not been validated here.

Activate the framework environment and source its ROS install in a bash terminal.
Check that these commands work before building navigation:

```bash
python3 -c 'import rclpy, cv2, cv_bridge, robomaster'
ros2 pkg prefix robomaster_ros
```

## Build this package

From this repository's root, with the dependency environment active:

```bash
python3 -m colcon build --base-paths . --packages-select robomaster_example
```

In a fresh bash terminal, activate the same dependency environment, source its
ROS install again, then source this package's overlay:

```bash
source install/setup.bash
```

The course workspace also includes a package named `robomaster_example`. Build
this repository separately and source its overlay last so the updated controller
is selected. Do not include both copies in one colcon source scan.

## Start navigation

The scene uses two simulated robots: `RM0` is the ground robot at `127.0.0.1`;
`RM1` at `127.0.1.1` holds the overhead camera. Both SDK servers need a bridge.
On macOS, configure the extra loopback address before opening the scene:

```bash
sudo ifconfig lo0 alias 127.0.1.1/8 up
```

Set the path to your installed CoppeliaSim executable, then run:

```bash
export COPPELIASIM_BIN=/Applications/coppeliaSim.app/Contents/MacOS/coppeliaSim
bash scripts/run_simulation.sh
```

The launcher starts the scene, connects the two bridges sequentially, waits for
camera frames, and starts navigation. Keep the scene running. Close CoppeliaSim
or press Ctrl+C to end the run. Logs and route images are saved under `outputs/`.
The helper has been syntax-checked; the successful local run used the original
workspace's diagnostic launcher with the same connection sequence.

If using Linux, configure the equivalent loopback alias and set
`COPPELIASIM_BIN` to the simulator's launch executable. That configuration is untested.

To remove the additional macOS address after all simulation sessions end:

```bash
sudo ifconfig lo0 -alias 127.0.1.1
```

## Parameters and failure checks

The node publishes `/rm0/cmd_vel` and consumes `/rm1/camera/image_color`.
Launch arguments: `name`, `camera_topic`, `update_mode`, `update_interval`
(default 3 seconds), and `margin` (6 extra pixels beyond the robot footprint).

- Missing `RM1`: check the loopback alias and the second bridge log.
- No frames: confirm the scene is running; video must remain continuously enabled.
- Marker detection failure: inspect `outputs/navigation/localization_failure.jpg`.
- No safe route: the controller stops rather than reducing obstacle clearance.
- Missing plugins: rebuild the framework plugins for the active environment.

The map refreshes at up to 2 Hz and pose feedback at up to 10 Hz. These rates do
not guarantee avoidance of arbitrary moving obstacles. Calibration assumes the
purple marker represents a 0.55 m square and remains valid for a fixed camera.
