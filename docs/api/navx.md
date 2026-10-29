# NavX API

## Overview

The `froglib.navx` module provides a specialized wrapper for the Kauai Labs NavX AHRS inertial measurement unit. Designed specifically for swerve drive systems and field-oriented control:

- **Counter-Clockwise Positive**: Standard NavX yaw outputs follow compass bearing conventions (clockwise positive). `FROGNavXGyro` inverts this telemetry to adhere to standard Cartesian/trigonometric conventions (counter-clockwise positive), matching WPILib kinematics and odometry standards.
- **Alliance-Aware Reorientation**: Allows resetting robot heading while automatically accounting for alliance color (0° heading for the Blue alliance driver station perspective, 180° heading for the Red alliance perspective).
- **Constrained Heading (`getAngleConstrained()`)**: Returns the heading constrained to the continuous `[-180, 180]` degree range, useful for angular error calculation and steering controllers.
- **WPILib Geometry Interoperability**: Directly exports robot heading as `wpimath.geometry.Rotation2d` objects for seamless integration into drivetrain pose estimators.
- **Compatibility Alias**: `FROGGyro` is provided as an alias for `FROGNavXGyro` (consolidating the sensor wrapper from legacy `froglib.sensors`).

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| [`FROGNavXGyro`][froglib.navx.FROGNavXGyro] | Class | Wraps the Kauai Labs NavX AHRS over SPI with CCW-positive angles, roll/pitch telemetry, `getAngleConstrained()`, and alliance-specific resets. |
| `FROGGyro` | Class Alias | Backward-compatibility alias pointing directly to `FROGNavXGyro`. |

## Usage Examples

### Instantiation and Heading Query

```python
from wpimath.geometry import Rotation2d
from froglib.navx import FROGNavXGyro  # or FROGGyro

# 1. Initialize NavX over SPI
navx = FROGNavXGyro()

# 2. Get current heading as degrees (CCW positive)
heading_deg = navx.getAngleCCW()

# 3. Get heading constrained to [-180, 180] degrees
constrained_deg = navx.getAngleConstrained()

# 4. Get heading as a WPILib Rotation2d object
heading_rot2d = navx.getRotation2d()

# 5. Get rotational velocity in radians/sec
rot_rate = navx.getRadiansPerSecCCW()
print(f"Heading: {heading_deg:.2f}°, Constrained: {constrained_deg:.2f}°, Rate: {rot_rate:.3f} rad/s")
```

### Alliance-Aware Heading Reset

```python
from wpilib import DriverStation
from froglib.navx import FROGNavXGyro

gyro = FROGNavXGyro()

# Determine if robot is on the Red alliance
is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed

# Reset gyro: sets 180° adjustment if on Red, 0° if on Blue
gyro.resetGyro(on_red=is_red)
```

## Reference

::: froglib.navx
