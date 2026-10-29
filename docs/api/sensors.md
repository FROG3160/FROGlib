# Sensors API

## Overview

The `froglib.sensors` module provides hardware abstraction wrappers for sensors commonly used across FRC robot mechanisms and drivetrains:

1. **Inertial Measurement / Gyro (`FROGGyro`)**: Wraps the Kauai Labs NavX AHRS over SPI. It converts raw yaw bearing angles into standard Cartesian angles where counter-clockwise (CCW) rotation is positive. It also provides alliance-aware reset functions (orienting 180° for the Red alliance and 0° for the Blue alliance) and conversion methods into WPILib `Rotation2d` objects for drivetrain kinematics.
2. **Absolute Magnetic Encoders (`FROGCanCoder`, `FROGCANCoderConfig`)**: Wraps the CTRE Phoenix 6 CANcoder. It preconfigures signed absolute sensor ranges (`[-0.5, 0.5)` rotations) and CCW-positive sensor directions while applying module-specific steering offsets for calibrated swerve azimith alignment.

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| [`FROGGyro`][froglib.sensors.FROGGyro] | Class | NavX AHRS SPI gyro wrapper standardizing CCW-positive heading, pitch/roll telemetry, and alliance resetting. |
| [`FROGCANCoderConfig`][froglib.sensors.FROGCANCoderConfig] | Class | Phoenix 6 `CANcoderConfiguration` subclass configured for signed range, CCW direction, and steer magnet offset. |
| [`FROGCanCoder`][froglib.sensors.FROGCanCoder] | Class | Phoenix 6 `CANcoder` subclass that applies `FROGCANCoderConfig` on instantiation. |

## Usage Examples

### Drivetrain Gyro Initialization & Alliance Reset

```python
from wpilib import DriverStation
from froglib.sensors import FROGGyro

# 1. Instantiate gyro (initializes NavX over SPI)
gyro = FROGGyro()

# 2. Reset heading based on driver station alliance color
is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
gyro.resetGyro(on_red=is_red)

# 3. Read heading and rotation velocity for swerve kinematics
heading_rot2d = gyro.getRotation2d()
angular_velocity_rad_per_sec = gyro.getRadiansPerSecCCW()
print(f"Robot Heading: {heading_rot2d.degrees():.2f}°, Rate: {angular_velocity_rad_per_sec:.2f} rad/s")
```

### Swerve Azimuth CANcoder Configuration

```python
from froglib.sensors import FROGCanCoder, FROGCANCoderConfig

# 1. Define calibrated steer offset in rotations (from calibration routine)
STEER_OFFSET_ROTATIONS = -0.1425

# 2. Create the configuration
cancoder_config = FROGCANCoderConfig(steer_offset=STEER_OFFSET_ROTATIONS)

# 3. Instantiate the CANcoder on CAN ID 21
azimuth_encoder = FROGCanCoder(id=21, config=cancoder_config)

# 4. Read absolute position in rotations
absolute_pos = azimuth_encoder.get_absolute_position().value_as_double
print(f"Module absolute steer position: {absolute_pos:.4f} rotations")
```

## Reference

::: froglib.sensors
