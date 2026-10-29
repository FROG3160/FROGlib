# Xbox API

## Overview

The `froglib.xbox` module provides specialized subclasses of WPILib's `CommandXboxController` tailored for driver and operator input in FRC:

1. **Driver Control (`FROGXboxDriver`)**: Designed for field-oriented swerve drivetrains. It features:
   - Configurable joystick deadbands to eliminate drift.
   - Slew rate limiting on translation and rotation to prevent mechanical stress, wheel slip, and brownouts.
   - Alliance-aware coordinate flipping: Inverts translation controls when on the Red Alliance so "pushing forward" always drives away from the driver's perspective regardless of alliance wall station.
   - Debounced D-pad (POV) trigger helpers with haptic rumble pulses on state changes.
2. **Tactical / Operator Control (`FROGXboxTactical`)**: Designed for mechanism manipulation (intakes, note indexers/transfers, leadscrews, flywheels), providing deadbanded velocity and trigger getters.

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| `RIGHT_RUMBLE` | Constant | Alias for `GenericHID.RumbleType.kRightRumble`. |
| `LEFT_RUMBLE` | Constant | Alias for `GenericHID.RumbleType.kLeftRumble`. |
| [`FROGXboxDriver`][froglib.xbox.FROGXboxDriver] | Class | Driver controller with deadbanding, slew rate limiters, alliance flipping, and debounced POV buttons. |
| [`FROGXboxTactical`][froglib.xbox.FROGXboxTactical] | Class | Operator controller with deadbanded stick and trigger getters for robot mechanisms. |

## Usage Examples

### Field-Oriented Swerve Driver Setup

```python
from wpilib import DriverStation
from froglib.xbox import FROGXboxDriver

# Initialize driver controller on USB port 0:
# deadband=0.1, debouncePeriod=0.5s, translationSlew=3.0 units/s, rotSlew=3.0 units/s
driver_ctrl = FROGXboxDriver(
    port=0,
    deadband=0.1,
    debouncePeriod=0.5,
    translationSlew=3.0,
    rotSlew=3.0
)

# Update alliance color in autonomousInit or teleopInit
driver_ctrl.set_alliance(DriverStation.getAlliance())

# Query slew-limited, alliance-adjusted velocities in teleop periodic
forward_velocity = driver_ctrl.getSlewLimitedFieldForward()
left_velocity = driver_ctrl.getSlewLimitedFieldLeft()
rotation_rate = driver_ctrl.getSlewLimitedFieldRotation()

# Bind debounced POV direction to preset commands
# driver_ctrl.pov0().onTrue(AlignToSpeakerCommand())
# driver_ctrl.pov180().onTrue(AlignToAmpCommand())
```

### Mechanism Operator Setup

```python
from froglib.xbox import FROGXboxTactical

# Initialize tactical controller on USB port 1 with 0.05 deadband
operator_ctrl = FROGXboxTactical(port=1, deadband=0.05)

# Poll mechanism speeds
intake_speed = operator_ctrl.getIntakeWheelSpeed()
shooter_speed = operator_ctrl.getFlyWheelSpeed()
```

## Reference

::: froglib.xbox
