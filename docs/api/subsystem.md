# Subsystem API

The `froglib.subsystem` module provides the [`FROGSubsystem`][froglib.subsystem.FROGSubsystem] base class, introducing declarative NetworkTables 4 telemetry logging and dashboard tunables to WPILib `commands2.Subsystem` implementations.

---

## Overview

Modern FRC robots require rapid parameter tuning during testing and comprehensive telemetry logging during matches. Traditionally, this required boilerplate getter/setter code, repeated NetworkTables publisher instantiations, and explicit logging calls in every subsystem's `periodic()` loop.

`froglib.subsystem` streamlines this workflow:

1. **Declarative Telemetry (`@telemetry`)**: Decorate any zero-argument method to automatically publish its return value to NetworkTables (`FROGSubsystems/<SubsystemName>/telemetry/<TopicName>`) and simultaneously append historical data to the WPILib `DataLogManager`.
2. **Dashboard Tunables (`@tunable`)**: Decorate a setter method to expose a tunable property under `FROGSubsystems/<SubsystemName>/tunables/<TopicName>`. Changes typed on Shuffleboard or AdvantageScope immediately invoke the setter method.
3. **Automated Reflection**: On initialization, `FROGSubsystem` scans its class for `@telemetry` and `@tunable` decorators, discovering all telemetry and tunable descriptors and registering TalonFX motors for diagnostics without manual registration.

---

## Key Components

| Component | Type | Description |
| :--- | :--- | :--- |
| `FROGSubsystem` | `class (Subsystem)` | Enhanced base subsystem class that automatically executes telemetry and tunable updates during each scheduler iteration. |
| `@tunable` | `decorator / descriptor` | Decorator that turns a method into a dashboard-tunable NetworkTables topic with optional callback execution on value changes. |
| `@telemetry` | `decorator / descriptor` | Decorator that turns a method into an automated telemetry getter published to NetworkTables and optionally logged to WPILib DataLog. |
| `Tunable` | `class (Descriptor)` | Python descriptor implementing bidirectional NetworkTables synchronization for `int`, `float`, `bool`, `str`, and `Pose2d`. |
| `Telemetry` | `class (Descriptor)` | Python descriptor managing periodic telemetry publishing and typed WPILib DataLog entries (`DoubleLogEntry`, `BooleanLogEntry`, `StringLogEntry`). |
| `Direction` | `Enum` | Enumeration representing mechanism motion (`FORWARD = 1`, `REVERSE = -1`, `IDLE = 0`). |

---

## Usage Examples

### Implementing a Subsystem with Tunables & Telemetry

```python
from froglib.subsystem import FROGSubsystem, tunable, telemetry
from froglib.ctre import FROGTalonFX, FROGTalonFXConfig

class FlywheelSubsystem(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(id=15, motor_name="FlywheelMotor")
        self._target_rps = 45.0

    @tunable(default=45.0, topic_name="Target RPS")
    def target_rps(self, new_rps: float) -> None:
        """Dashboard-tunable target velocity setpoint."""
        self._target_rps = new_rps

    @telemetry(topic_name="Actual RPS", log=True)
    def actual_rps(self) -> float:
        """Current flywheel velocity logged to DataLog and NetworkTables."""
        return self.motor.get_velocity().value

    @telemetry(topic_name="At Speed", log=False)
    def is_at_speed(self) -> bool:
        """Boolean indicator showing if flywheel is within 2 RPS of setpoint."""
        return abs(self.actual_rps - self._target_rps) < 2.0

    def periodic(self) -> None:
        # Crucial: super().periodic() updates all @telemetry and @tunable descriptors
        super().periodic()
```

### Accessing Tunable Properties

```python
flywheel = FlywheelSubsystem()

# Read the current tunable value (reflects latest dashboard edits)
current_target = flywheel.target_rps

# Set a new value from autonomous or command code (publishes to NetworkTables)
flywheel.target_rps = 60.0
```

---

## Reference

::: froglib.subsystem
