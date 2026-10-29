# Swerve API

The `froglib.swerve` module provides high-level control abstractions, kinematics, and odometry pose estimation for four-wheel independent swerve drivetrains.

---

## Overview

A swerve drive allows an FRC robot to translate in any direction while independently controlling chassis rotation. `froglib.swerve` coordinates this through two main classes:

1. **[`SwerveModule`][froglib.swerve.SwerveModule]**: Controls a single wheel assembly consisting of a drive motor ([`FROGTalonFX`][froglib.ctre.FROGTalonFX]), an azimuth steering motor ([`FROGTalonFX`][froglib.ctre.FROGTalonFX]), and a steering encoder ([`FROGCanCoder`][froglib.ctre.FROGCanCoder]). It continuously handles azimuth angle optimization (reversing drive direction if the turn angle exceeds 90°) and sets closed-loop velocity and position commands.
2. **[`SwerveChassis`][froglib.swerve.SwerveChassis]**: Coordinates all four modules using WPILib's `SwerveDrive4Kinematics` and `SwerveDrive4PoseEstimator`. It provides field-oriented driving (`fieldOrientedDrive()` with optional rotation throttling via `throttle_rotation: bool = True`), robot-centric driving, autonomous heading tracking via a `ProfiledPIDControllerRadians`, chassis wheel locking (X-brake), and comprehensive NetworkTables telemetry.

---

## Key Components

| Class | Base Class | Description |
| :--- | :--- | :--- |
| `SwerveChassis` | `object` | Kinematics and odometry manager coordinating four swerve modules and a Pigeon 2.0 gyro; supports field-oriented drive with `throttle_rotation` control. |
| `SwerveModule` | `object` | Individual swerve module controller handling drive velocity, steer position, and azimuth optimization. |
| `SwerveModuleConfig` | `dataclass` | Configuration structure specifying motor IDs, CANcoder ID, motor/sensor configs, location, and wheel diameter. |
| `RotationControllerConfig` | `object` | PID gains and trapezoidal motion profiling constraints for autonomous rotation control. |

---

## Usage Examples

### Configuring and Instantiating Swerve Modules

```python
from wpimath.geometry import Translation2d
from wpimath.units import inchesToMeters
from froglib.swerve import SwerveModuleConfig, SwerveChassis, RotationControllerConfig
from froglib.ctre import FROGPigeonGyro, get_frog_talon_config, get_frog_cancoder_config
from froglib.sds import MK4C_L3_GEARING, WHEEL_DIAMETER

# Half-track width and half-wheel base in meters
kTrackWidth = inchesToMeters(22.0)
kWheelBase = inchesToMeters(22.0)

# Define physical module configurations (FL, FR, BL, BR)
fl_config = SwerveModuleConfig(
    name="FrontLeft",
    location=Translation2d(kWheelBase / 2, kTrackWidth / 2),
    drive_motor_id=1,
    steer_motor_id=2,
    cancoder_id=3,
    wheel_diameter=WHEEL_DIAMETER,
)

fr_config = SwerveModuleConfig(
    name="FrontRight",
    location=Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    drive_motor_id=4,
    steer_motor_id=5,
    cancoder_id=6,
    wheel_diameter=WHEEL_DIAMETER,
)

bl_config = SwerveModuleConfig(
    name="BackLeft",
    location=Translation2d(-kWheelBase / 2, kTrackWidth / 2),
    drive_motor_id=7,
    steer_motor_id=8,
    cancoder_id=9,
    wheel_diameter=WHEEL_DIAMETER,
)

br_config = SwerveModuleConfig(
    name="BackRight",
    location=Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    drive_motor_id=10,
    steer_motor_id=11,
    cancoder_id=12,
    wheel_diameter=WHEEL_DIAMETER,
)
```

### Initializing and Driving the Chassis

```python
# Gyro and Profiled Rotation Controller
gyro = FROGPigeonGyro(can_id=0)

rotation_config = RotationControllerConfig(
    kProfiledRotationP=4.5,
    kProfiledRotationI=0.0,
    kProfiledRotationD=0.1,
    kProfiledRotationMaxVelocity=10.0,  # rad/s
    kProfiledRotationMaxAccel=20.0,     # rad/s²
)

# Instantiate chassis
chassis = SwerveChassis(
    swerve_module_configs=(fl_config, fr_config, bl_config, br_config),
    gyro=gyro,
    rotation_contoller_config=rotation_config,
    max_speed=4.5,          # m/s
    max_rotation_speed=10.0, # rad/s
    parent_nt="Drivetrain",
)

chassis.enable()

# Driving during teleop periodic:
# Command forward 80% speed, 20% left drift, no rotation, at 100% throttle.
# throttle_rotation (default True) determines whether rotation is scaled with translation throttle:
chassis.fieldOrientedDrive(vX=0.8, vY=0.2, vT=0.0, throttle=1.0, throttle_rotation=True)

# In autonomous or when holding position:
# chassis.lockChassis()
```

---

## Reference

::: froglib.swerve
