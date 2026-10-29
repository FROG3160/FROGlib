# Motors API

The `froglib.motors` module provides motor controller wrappers for CTRE Phoenix 5 and Phoenix 6 hardware, alongside mechanical drivetrain and gear ratio calculation utilities.

---

## Overview

Historically, `froglib.motors` served as FROG 3160's primary motor hardware abstraction layer, encapsulating TalonFX (Falcon 500 / Kraken X60) and legacy TalonSRX speed controllers with automatic NetworkTables telemetry logging.

!!! tip "Modern Alternative: `froglib.ctre`"
    For **new 2026/2027 FRC robot code**, teams should prefer [`froglib.ctre`](ctre.md). The `froglib.ctre` module provides:
    
    * Preconfigured frame-aligned CAN bus signal update frequency profiles (`FROGTalonFX.SignalProfile`).
    * Built-in empirical system identification and gain tuning (`TalonFXCharacterizer`).
    * First-class integration with WPILib `SysIdRoutine` and Phoenix 6 `SignalLogger`.
    * High-fidelity physics simulation (`DCMotorSim`).

Similarly, for modern gear stage representations and drivetrain conversions, prefer [`froglib.utils.GearStage`](utils.md) and [`froglib.utils.DriveTrain`](utils.md).

---

## Key Components

| Class | Base Class | Description |
| :--- | :--- | :--- |
| `FROGTalonFX` | `phoenix6.hardware.TalonFX` | Phoenix 6 TalonFX wrapper with automatic NetworkTables publishing for velocity, position, and voltage. |
| `FROGTalonFXConfig` | `phoenix6.configs.TalonFXConfiguration` | Convenience configuration class that accepts feedback configs and Slot 0 / Slot 1 gains in its constructor. |
| `FROGFeedbackConfig` | `phoenix6.configs.FeedbackConfigs` | Phoenix 6 feedback configuration helper allowing inline sensor source and remote sensor CAN ID assignment. |
| `FROGTalonSRX` | `phoenix5.TalonSRX` | Legacy Phoenix 5 TalonSRX wrapper with NetworkTables telemetry and 250 ms status frame rate optimization. |
| `FROGTalonSRXConfig` | `phoenix5.TalonSRXConfiguration` | Phoenix 5 TalonSRX configuration class. |
| `GearTrain` | `object` | Multi-stage reduction calculator that computes total gear ratio, mechanism input rotations, and output rotations. |
| `DriveTrain` | `object` | Combines a `GearTrain` with wheel circumference to convert linear robot speed (m/s) to motor velocity (RPS) and wheel distance. |

---

## Usage Examples

### Configuring a Legacy TalonFX Motor

```python
from phoenix6.configs import Slot0Configs
from froglib.motors import FROGTalonFX, FROGTalonFXConfig, FROGFeedbackConfig
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue

# Define slot gains
slot0 = Slot0Configs()
slot0.k_p = 0.25
slot0.k_v = 0.12

# Create feedback config using internal rotor sensor
feedback = FROGFeedbackConfig(
    remote_sensor_id=0,
    sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
)

# Apply configuration during motor instantiation
motor_config = FROGTalonFXConfig(
    feedback_config=feedback,
    slot0gains=slot0,
)

shooter_motor = FROGTalonFX(
    id=12,
    motor_config=motor_config,
    parent_nt="ShooterSubsystem",
    motor_name="FlywheelLead",
)

# Periodically publish telemetry to NetworkTables
shooter_motor.logData()
```

### Drivetrain Velocity & Distance Calculations

```python
from froglib.motors import DriveTrain

# Define gear stages: 10:32 first stage, 9:24 second stage
gear_stages = [(10 / 32), (9 / 24)]
wheel_diameter_meters = 0.1016  # 4 inches

drivetrain = DriveTrain(gear_stages, wheel_diameter_meters)

# Convert linear robot speed to motor RPS
motor_rps = drivetrain.speed_to_input_rps(3.0)  # 3.0 m/s
print(f"Target motor velocity: {motor_rps:.2f} RPS")

# Convert motor rotations to distance traveled
meters_traveled = drivetrain.rotations_to_distance(120.0)
print(f"Distance traveled: {meters_traveled:.2f} meters")
```

---

## Reference

::: froglib.motors
