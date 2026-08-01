# Indoor Mapping Rover

A low-cost, DIY indoor mapping rover built around ROS 2 Jazzy and dual ESP32s running micro-ROS. The rover uses a two-wheel differential drive with encoder feedback, basic ultrasonic sensing, manual control, and telemetry to build up a room map over time.

This is an active, in-progress student project. This README reflects the actual current state, not the target end state. See Status below.

---

## Status

**Mechanical / URDF: working**
- Full 3D model in RViz: base plate, two driven wheels, and a passive caster.
- Wheel geometry (`wheel_axle_x`, `wheel_separation`) and caster mesh alignment were debugged and corrected against real SolidWorks measurements, not just eyeballed in RViz. See [`docs/urdf_debugging_log.md`](docs/urdf_debugging_log.md) for the full process, including the actual bugs found and fixed.
- Assembly-relative STL export is now the standard workflow for any new mesh work.

**Simulation (Gazebo): working, diff-drive + SLAM both validated**
- The rover spawns into Gazebo Harmonic using `ros2_control` and `gz_ros2_control`, with `diff_drive_controller` and `joint_state_broadcaster` both loading and activating cleanly.
- Sending velocity commands actually drives the simulated wheels, and `/diff_drive_controller/odom` publishes real, correctly integrating pose data (position increasing as expected, timestamps advancing properly).
- **A real, unresolved upstream bug in Gazebo Harmonic's sensor rendering pipeline under WSL2** (`gz-sim`'s `Sensors` system hangs indefinitely at "Waiting for init", tracked in multiple open GitHub issues against `gz-sim`/`gz-rendering`, no fix as of this writing) blocks Gazebo's native LiDAR sensor from producing data on this dev machine. Ruled out during debugging: render engine choice (ogre vs. ogre2), missing shared library dependencies, EGL initialization, WSLg display passthrough, and CPU thread contention. Worked around with a synthetic ray-casting `/scan` publisher (`scripts/fake_lidar_publisher.py`) that computes geometrically accurate LaserScan data against the known test world geometry, using the robot's real (validated) odometry pose. `slam_toolbox` was then validated end-to-end against this synthetic scan data, building a correct occupancy grid map in RViz matching the test world's actual walls and obstacles.
- Full debugging trail in [`docs/urdf_debugging_log.md`](docs/urdf_debugging_log.md).

**Firmware (real hardware): skeleton plus a working PID controller, no odometry yet**
- PlatformIO project structure is in place (`firmware/`).
- Wheel velocity PID control is implemented, with anti-windup handling, running in a 1kHz FreeRTOS task pinned to a core.
- Encoder-to-pose odometry math on the physical ESP32 (the equivalent of what's already validated in simulation) has not been written yet.

**Sensing: not yet integrated (hardware)**
- SG90 servo and HC-SR04 ultrasonic sensor are planned but not yet wired into a working pipeline on physical hardware. Simulation-side sensing (LiDAR + SLAM) is validated, see above.

**Mapping / dashboard: validated in simulation, not started on hardware**
- Simulated mapping pipeline (LiDAR-equivalent + `slam_toolbox`) is working and validated, see Simulation status above.
- Real-hardware mapping depends on the ultrasonic sensing pipeline being built out first.

---

## Architecture

**Simulation (working now):**
```
ros2 topic pub /cmd_vel -> diff_drive_controller (ros2_control) -> Gazebo Harmonic physics
|
v
/diff_drive_controller/odom (real, validated pose data)
|
v
synthetic /scan (ray-cast vs. known world geometry)
[workaround for a WSL2 Gazebo sensor
rendering bug -- see Status above]
|
v
slam_toolbox (sync mode) -> /map (validated, real occupancy grid)

Simulation deliberately uses LiDAR + SLAM, not ultrasonic, since sim has no hardware cost constraint.
```

**Physical hardware (target, partially built):**
[ESP32 #1: drive] [ESP32 #2: sensing]
encoders SG90 + HC-SR04
motor PID (done) |
| |
└────── micro-ROS / WiFi UDP ──────┐
v
ROS 2 Jazzy (host)
odometry, mapping

The physical architecture is the eventual target. Right now the drive-side control loop (PID) is implemented and running on the ESP32, but odometry and the micro-ROS bridge to ROS 2 are not yet built. The simulation stack, by contrast, is fully working end to end, including SLAM.

---

## Repo layout
rover_description/ ROS 2 package: URDF/xacro, meshes, launch files, worlds, scripts
firmware/ PlatformIO project for ESP32 firmware
docs/ Debugging logs and design notes

---

## What's next

Roughly in priority order:

1. Build a second, realistic Gazebo world matching an actual measured living space, and re-run the validated SLAM pipeline against it as a stronger portfolio piece.
2. Port odometry math to the real ESP32 firmware, using the simulation's validated behavior as a reference for what correct output should look like.
3. Build out the ultrasonic sensing pipeline on real hardware.
4. Validate the real hardware with tests: straight-line distance accuracy and 90 degree turn heading drift.
5. Add a web dashboard and live telemetry, time permitting.
6. Investigate whether Gazebo's native LiDAR rendering can be un-blocked on WSL2 (newer WSLg release, or testing outside WSL), in case the synthetic scan workaround is no longer needed.

## Known limitations

- **No IMU yet.** Odometry currently relies on wheel encoders alone (or their simulated equivalent), which drifts over time and through turns. Adding an IMU for sensor fusion is a candidate future upgrade.
- **Simulation's LiDAR data is synthetic, not rendered by Gazebo.** Due to an unresolved Gazebo Harmonic sensor-rendering bug under WSL2, `/scan` in simulation is produced by ray-casting against known world geometry rather than Gazebo's actual sensor pipeline. The scan data is geometrically accurate and slam_toolbox has been validated against it, but this is a workaround, not native sensor simulation.
- **No LiDAR on physical hardware.** The real rover's mapping still relies on a single ultrasonic sensor on a servo sweep, so real-world maps will be noisier and lower-resolution than the simulated LiDAR-based system. A 2D LiDAR on real hardware is a possible future upgrade once the current ultrasonic pipeline is validated.