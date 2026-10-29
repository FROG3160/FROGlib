# Welcome to FROGlib

**FROGlib** is a high-level hardware abstraction and utility library developed for FIRST Robotics Competition (FRC) Team 3160, the **FROGs**. Built on top of **RobotPy**, **CTRE Phoenix 6**, and **WPILib**, FROGlib accelerates robot software development by providing robust hardware wrappers, declarative subsystem telemetry, swerve drive kinematics, multi-tag AprilTag vision pose estimation, and automated on-robot motor characterization and gain tuning.

---

## Features

- **Automated CTRE Motor Characterization & Tuning**: Autonomous on-robot determination of $k_S, k_V, k_A$, continuous 2nd-order pole placement synthesis of feedback gains $k_P, k_D$, automated gearing validation, travel boundary protection, and WPILib `SysIdRoutine` integration with 250 Hz Phoenix 6 `SignalLogger` (`.hoot` logs).
- **CTRE Phoenix 6 Hardware & Configuration Wrappers**: All CTRE hardware wrappers and configuration classes (`FROGTalonFX`, `FROGTalonFXConfig`, `FROGFeedbackConfig`, `FROGCanCoder`, `FROGCANCoderConfig`, `FROGPigeonGyro`) are consolidated in `froglib.ctre`, featuring `SignalProfile` CAN bus frame rate optimization (reducing bus utilization from >85% down to <45%), WPILib `DCMotorSim` physics simulation, and SysId integration.
- **Declarative Subsystem Architecture**: `FROGSubsystem` with `@telemetry` and `@tunable` descriptors that automatically bind Python methods and properties to NetworkTables 4 and WPILib `DataLogManager` (.wpilog) with zero boilerplate.
- **Swerve Drive Kinematics & Pose Estimation**: Multi-module `SwerveChassis` coordinator with heading stabilization (`RotationControllerConfig`), `SwerveModule` coordination, and verified gear ratio presets for Swerve Drive Specialties MK4c and MK5i modules.
- **Vision & Game-Piece Detection**: `FROGPoseEstimator` for multi-tag AprilTag pose estimation with standard deviation filtering and `PoseBuffer`, plus `FROGDetector` with numpy-based spatial clustering for game-piece tracking.
- **Driver & Operator Controls**: `FROGXboxDriver` and `FROGXboxTactical` controllers offering customizable response curves, exponential shaping, deadbands, debounced triggers, and rumble feedback.
- **Sensors, Field Geometry & Utilities**: IMU abstraction with `FROGNavXGyro` (and alias `FROGGyro`) in `froglib.navx`, alliance coordinate mapping with `FROGField`, plus `DriveTrain`, `GearStage`, and `RobotRelativeTarget` geometric math utilities in `froglib.utils`.

---

## Module Overview

The library is organized into the following modules:

| Module | Primary Classes & Components | Description |
| :--- | :--- | :--- |
| **`froglib.ctre`** | `FROGTalonFX`, `FROGTalonFXConfig`, `FROGFeedbackConfig`, `FROGCanCoder`, `FROGCANCoderConfig`, `FROGPigeonGyro`, `TalonFXCharacterizer`, `FROGSysIdHelper`, `get_frog_talon_config`, `get_frog_cancoder_config` | Consolidated CTRE Phoenix 6 wrappers and configuration classes, CAN bus signal profiles, physics simulation, automated gain tuning, and SysId integration. See the [Tuning Guide](tuning_guide.md) and [Testing Guide](testing_tuning.md). |
| **`froglib.subsystem`** | `FROGSubsystem`, `@telemetry`, `@tunable`, `Direction` | Base subsystem class with declarative NetworkTables 4 publishers and live tunable dashboard properties. |
| **`froglib.swerve`** | `SwerveChassis`, `SwerveModule`, `SwerveModuleConfig`, `RotationControllerConfig` | Full swerve drive chassis coordinator, odometry, heading lock PID, and module steering alignment. |
| **`froglib.sds`** | `MK4C_L3_GEARING`, `MK5I_R1_GEARING`, `MK5I_R2_GEARING`, `MK5I_R3_GEARING`, `WHEEL_DIAMETER` | Swerve Drive Specialties MK4c and MK5i module gear ratio definitions and wheel dimensions. |
| **`froglib.vision`** | `FROGPoseEstimator`, `FROGDetector`, `FROGCameraConfig`, `DetectionResult` | PhotonVision multi-tag AprilTag pose estimation, standard deviation filtering, and numpy game-piece clustering. |
| **`froglib.xbox`** | `FROGXboxDriver`, `FROGXboxTactical` | Extended Xbox controllers for driver and tactical operators with curve shaping and debouncing. |
| **`froglib.navx`** | `FROGNavXGyro`, `FROGGyro` | Kauai Labs NavX IMU wrapper in `froglib.navx` providing gyro yaw, pitch, roll, heading reset, and constrained angle retrieval (`FROGGyro` is an alias). |
| **`froglib.field`** | `FROGField` | Field coordinate transformations, POIs, and red/blue alliance flipping. |
| **`froglib.utils`** | `DriveTrain`, `GearStage`, `Buffer`, `PoseBuffer`, `RobotRelativeTarget` | Gearbox ratio calculators, drivetrain speed/distance conversion, circular buffers, and robot-relative polar/Cartesian target calculations. |

---

## Next Steps

- Follow the [Getting Started](getting_started.md) guide to install FROGlib and set up your first subsystem.
- Learn how to tune motor gains in the [TalonFX Tuning Guide](tuning_guide.md).
- Verify simulation and robot safety in [Testing & Verifying Motor Tuning](testing_tuning.md).
- Explore the API Reference in the sidebar for full class signatures and docstrings.
