# Indoor Mapping Rover

A low-cost, DIY indoor mapping rover built around ROS 2 Jazzy and dual ESP32s running micro-ROS. The rover uses a two-wheel differential drive with encoder feedback, basic ultrasonic sensing, manual control, and telemetry to build up a room map over time.

This is an active, in-progress student project. This README reflects the actual current state, not the target end state. See Status below.

---

## Status

**Mechanical / URDF: working**
- Full 3D model in RViz: base plate, two driven wheels, and a passive caster.
- Wheel geometry (`wheel_axle_x`, `wheel_separation`) and caster mesh alignment were debugged and corrected against real SolidWorks measurements, not just eyeballed in RViz. See [`docs/urdf_debugging_log.md`](docs/urdf_debugging_log.md) for the full process, including the actual bugs found and fixed.
- Assembly-relative STL export is now the standard workflow for any new mesh work.

**Firmware: skeleton only, not yet functional**
- PlatformIO project structure is in place (`firmware/`).
- No odometry, differential-drive math, or encoder integration is implemented yet. This is the next milestone, not a hidden gap, confirmed directly against the source rather than assumed.

**Sensing: not yet integrated**
- SG90 servo and HC-SR04 ultrasonic sensor are planned but not yet wired into a working pipeline.

**Mapping / dashboard: not started**
- Planned, and depends on firmware odometry actually working first.

---

## Architecture (planned)

```
[ESP32 #1: drive]        [ESP32 #2: sensing]
   encoders                  SG90 + HC-SR04
   motor PID                     |
       |                         |
       └────── micro-ROS / WiFi UDP ──────┐
                                            v
                                    ROS 2 Jazzy (host)
                                    odometry, mapping
```

This is the target architecture, not a description of working code yet.

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

1. Implement encoder-based odometry and differential-drive math in firmware. This is currently missing entirely, not just buggy.
2. Validate with real tests: straight-line distance accuracy and 90 degree turn heading drift.
3. Integrate ultrasonic sensing and build a basic occupancy-grid mapping pipeline from logged pose and range data.
4. Add a web dashboard and live telemetry, time permitting.
5. Down the line, add a 2D LiDAR and a camera for proper SLAM and object detection. The current ultrasonic-plus-encoder setup is a starting point, not the final architecture.

## Known limitations

- **No IMU yet.** Odometry currently relies on wheel encoders alone, which drifts over time and through turns. Adding an IMU for sensor fusion is a candidate future upgrade.
- **No LiDAR yet.** Mapping currently relies on a single ultrasonic sensor on a servo sweep, so maps will be noisier and lower-resolution than a LiDAR-based system. A 2D LiDAR is planned for a future iteration once the current pipeline is validated.
- **Firmware odometry and diff-drive math aren't implemented yet.** Any timing, accuracy, or drift numbers referenced elsewhere (like a resume) that imply a working, validated system should be checked against this repo before trusting them.
