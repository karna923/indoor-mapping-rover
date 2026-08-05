# Indoor Mapping Rover

A low-cost, DIY indoor mapping rover built around ROS 2 Jazzy and dual ESP32s running micro-ROS. The rover uses a two-wheel differential drive with encoder feedback, basic ultrasonic sensing, manual control, and telemetry to build up a room map over time.

This is an active, in-progress student project. This README reflects the actual current state, not the target end state. See Status below.

---

## Status

**Mechanical / URDF: working**
- Full 3D model in RViz: base plate, two driven wheels, and a passive caster.
- Wheel geometry (`wheel_axle_x`, `wheel_separation`) and caster mesh alignment were debugged and corrected against real SolidWorks measurements, not just eyeballed in RViz. See [`docs/urdf_debugging_log.md`](docs/urdf_debugging_log.md) for the full process, including the actual bugs found and fixed.
- Assembly-relative STL export is now the standard workflow for any new mesh work.

**Simulation (Gazebo): working**
- The rover spawns into Gazebo Harmonic using `ros2_control` and `gz_ros2_control`, with `diff_drive_controller` and `joint_state_broadcaster` both loading and activating cleanly.
- Sending velocity commands actually drives the simulated wheels, and `/diff_drive_controller/odom` publishes real, correctly integrating pose data (position increasing as expected, timestamps advancing properly).
- This is genuinely validated, not just "it launched": position, velocity, and timing were all checked directly against expected values before calling it done.

**Firmware (real hardware): skeleton plus a working PID controller, no odometry yet**
- PlatformIO project structure is in place (`firmware/`).
- Wheel velocity PID control is implemented, with anti-windup handling, running in a 1kHz FreeRTOS task pinned to a core.
- Encoder-to-pose odometry math on the physical ESP32 (the equivalent of what's already validated in simulation) has not been written yet. That's the next real milestone on the hardware side.

**Sensing: not yet integrated**
- SG90 servo and HC-SR04 ultrasonic sensor are planned but not yet wired into a working pipeline, in sim or on hardware.

**Mapping / dashboard: not started**
- Planned, and depends on either the simulated or real sensing pipeline being in place first.

---

## Architecture

**Simulation (working now):**
```
ros2 topic pub /cmd_vel  ->  diff_drive_controller (ros2_control)  ->  Gazebo Harmonic physics
                                        |
                                        v
                              /diff_drive_controller/odom  (real, validated pose data)
```

**Physical hardware (target, partially built):**
```
[ESP32 #1: drive]        [ESP32 #2: sensing]
   encoders                  SG90 + HC-SR04
   motor PID (done)               |
       |                          |
       └────── micro-ROS / WiFi UDP ──────┐
                                            v
                                    ROS 2 Jazzy (host)
                                    odometry, mapping
```

The physical architecture is the eventual target. Right now the drive-side control loop (PID) is implemented and running on the ESP32, but odometry and the micro-ROS bridge to ROS 2 are not yet built. The simulation stack, by contrast, is fully working end to end.

---

## Repo layout

```
rover_description/   ROS 2 package: URDF/xacro, meshes, launch files
firmware/             PlatformIO project for ESP32 firmware
docs/                 Debugging logs and design notes
```

---

## What's next

Roughly in priority order:

1. Add a simulated ultrasonic sensor and start building the mapping pipeline against simulated data, since the drive and odometry side of the simulation is now solid.
2. Port odometry math to the real ESP32 firmware, using the simulation's validated behavior as a reference for what correct output should look like.
3. Validate the real hardware with tests: straight-line distance accuracy and 90 degree turn heading drift.
4. Add a web dashboard and live telemetry, time permitting.
5. Down the line, add a 2D LiDAR and a camera for proper SLAM and object detection. The current ultrasonic-plus-encoder setup is a starting point, not the final architecture.

## Known limitations

- **No IMU yet.** Odometry currently relies on wheel encoders alone (or their simulated equivalent), which drifts over time and through turns. Adding an IMU for sensor fusion is a candidate future upgrade.
- **No LiDAR yet.** Mapping currently relies on a single ultrasonic sensor on a servo sweep, so maps will be noisier and lower-resolution than a LiDAR-based system. A 2D LiDAR is planned for a future iteration once the current pipeline is validated.
- **Odometry is validated in simulation, not yet on real hardware.** The Gazebo pipeline confirms the approach works correctly, but the ESP32 firmware still needs its own encoder-to-pose math written and tested on the physical rover before any real-world accuracy numbers exist.
- Any timing, accuracy, or drift numbers referenced elsewhere (like a resume) that imply a working, validated system on physical hardware should be checked against this repo before trusting them.

The goal here is to be upfront about what's actually working versus what isn't, rather than make the project look more finished than it is.
