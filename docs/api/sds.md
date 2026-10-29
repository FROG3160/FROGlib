# SDS Module API

The `froglib.sds` module defines standardized gear train reductions and physical dimensions for popular **Swerve Drive Specialties (SDS)** swerve modules, including the MK4c and MK5i (inverted) models.

---

## Overview

Accurate robot localization and velocity control depend on precise conversions between motor rotor rotations and linear wheel distance traveled. The `froglib.sds` module predefines these mechanical gear stages using [`GearStage`](utils.md) objects.

When combined with [`DriveTrain`](utils.md) or passed into [`SwerveModuleConfig`](swerve.md), these constants eliminate manual calculation errors when configuring TalonFX `sensor_to_mechanism_ratio` feedback parameters and drive odometry.

---

## Key Constants

| Constant | Type | Reduction Ratio | Description |
| :--- | :--- | :--- | :--- |
| `MK4C_L3_GEARING` | `list[GearStage]` | 6.12 : 1 | Gear stages for the SDS MK4c Level 3 swerve module (Stages: 16:50, 28:16, 15:45). |
| `MK5I_R1_GEARING` | `list[GearStage]` | 8.14 : 1 | Gear stages for the SDS MK5i Ratio 1 swerve module (Stages: 12:54, 32:25, 15:30). |
| `MK5I_R2_GEARING` | `list[GearStage]` | 6.75 : 1 | Gear stages for the SDS MK5i Ratio 2 swerve module (Stages: 14:54, 32:25, 15:30). |
| `MK5I_R3_GEARING` | `list[GearStage]` | 5.91 : 1 | Gear stages for the SDS MK5i Ratio 3 swerve module (Stages: 16:54, 32:25, 15:30). |
| `WHEEL_DIAMETER` | `float` | N/A | Nominal diameter of standard 4-inch SDS wheels in meters (`~0.1016 m`). |

---

## Usage Examples

### Calculating Drive Train Gearing and Kinematics

```python
from froglib.sds import MK4C_L3_GEARING, WHEEL_DIAMETER
from froglib.utils import DriveTrain

# Create a DriveTrain helper for MK4c L3 swerve module
drivetrain = DriveTrain(MK4C_L3_GEARING, WHEEL_DIAMETER)

# Convert desired chassis speed (e.g., 3.5 m/s) to motor rotations per second
target_motor_rps = drivetrain.speed_to_input_rps(3.5)

# Convert measured motor velocity back to linear speed (m/s)
actual_mps = drivetrain.input_rps_to_speed(target_motor_rps)

# Convert encoder distance (motor rotations) to distance traveled in meters
distance_traveled_m = drivetrain.rotations_to_distance(50.0)
```

### Configuring Phoenix 6 TalonFX for SDS Swerve

```python
from phoenix6.configs import TalonFXConfiguration
from froglib.sds import MK5I_R2_GEARING
from froglib.utils import GearTrain

# Calculate total gear reduction
geartrain = GearTrain(MK5I_R2_GEARING)
gear_ratio = 1.0 / geartrain.gear_ratio  # Reduction factor (rotor : mechanism)

# Configure TalonFX sensor_to_mechanism_ratio
config = TalonFXConfiguration()
config.feedback.sensor_to_mechanism_ratio = gear_ratio
```

---

## Reference

::: froglib.sds
