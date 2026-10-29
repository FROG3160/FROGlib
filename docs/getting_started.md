# Getting Started with FROGlib

This guide walks you through installing FROGlib in your RobotPy project and building your first subsystem with hardware wrappers, declarative telemetry, and automated motor characterization.

---

## 1. Installation

FROGlib supports two active release tracks depending on your project needs:

- **`2026.1.2` (Latest Stable 2026 Release)**: Production-ready release for the 2026 competition season.
- **`2027.0.0a1` (2027 Alpha Pre-Release)**: Cutting-edge track featuring automated CTRE TalonFX characterization (`TalonFXCharacterizer`), WPILib SysId integration (`FROGSysIdHelper`), and physics simulation plants.

### Installing via pip

Install directly from GitHub into your RobotPy virtual environment:

```bash
# Stable 2026 release
pip install git+https://github.com/FROG3160/FROGlib.git@2026.1.2

# 2027 Alpha pre-release
pip install git+https://github.com/FROG3160/FROGlib.git@2027.0.0a1
```

### Specifying Dependencies in `pyproject.toml`

Add FROGlib to your robot project's dependencies:

```toml
[project]
name = "robot2026"
version = "0.1.0"
dependencies = [
    "robotpy>=2026.2.2",
    "phoenix6>26.0.0",
    # Pin to the stable 2026 track:
    "froglib @ git+https://github.com/FROG3160/FROGlib.git@2026.1.2",
    # OR pin to the 2027 alpha track:
    # "froglib @ git+https://github.com/FROG3160/FROGlib.git@2027.0.0a1",
]
```

### Local Editable Install (Development)

If you are developing FROGlib locally or testing changes alongside your robot code:

```bash
git clone https://github.com/FROG3160/FROGlib.git
cd FROGlib
pip install -e .
```

### Building Wheels for roboRIO Deployment

When deploying to a roboRIO without internet access, build a Python wheel (`.whl`) and distribute it with your project:

```bash
# Install the build frontend
pip install --upgrade build

# Build source distribution and binary wheel
python3 -m build
```

This generates `.whl` and `.tar.gz` packages in the `dist/` directory (e.g., `dist/froglib-2027.0.0a1-py3-none-any.whl`). You can copy this file to your robot project and install it on the roboRIO via RobotPy:

```bash
# Install the wheel into your roboRIO environment
pip install dist/froglib-*.whl
```

---

## 2. Hardware Configuration: `FROGTalonFX`

The `FROGTalonFX` wrapper extends CTRE Phoenix 6's `TalonFX` with `SignalProfile` CAN bus frame rate optimization, reducing bus utilization while ensuring high update rates where needed:

```python
from froglib.ctre import FROGTalonFX

# Instantiate a motor on CANivore with a FLYWHEEL signal profile
flywheel_motor = FROGTalonFX(
    id=15,
    signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
    canbus="canivore",
)

# Configure mechanism gear reduction (rotor turns per mechanism turn)
flywheel_motor.config.feedback.sensor_to_mechanism_ratio = 1.0

# Apply configuration changes
flywheel_motor.apply_config()
```

Available `SignalProfile` options include:
- `BASIC`: Low-frequency 50–100 Hz status signals for non-critical mechanisms.
- `FLYWHEEL`: 250 Hz velocity signal with suppressed position and acceleration frames.
- `POSITION_MM`: 250 Hz position frame for elevators and arms using Motion Magic.
- `SWERVE_DRIVE` / `SWERVE_AZIMUTH`: High-frequency odometry frames synchronized with CANivore.
- `TUNING`: Temporary high-frequency mode activated during characterization routines.

---

## 3. Declarative Subsystems: `FROGSubsystem`

Inheriting from `FROGSubsystem` provides automatic NetworkTables 4 publishing and WPILib `DataLogManager` logging using the `@telemetry` and `@tunable` descriptors:

```python
from froglib.subsystem import FROGSubsystem
from froglib.ctre import FROGTalonFX

class FlywheelSubsystem(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=15,
            signal_profile=FROGTalonFX.SignalProfile.FLYWHEEL,
        )

    # Telemetry: automatically published to NetworkTables under:
    # FROGSubsystems/FlywheelSubsystem/telemetry/velocity_rps
    # and recorded to DataLogManager (.wpilog)
    @FROGSubsystem.telemetry()
    def velocity_rps(self) -> float:
        return self.motor.get_velocity().value

    # Tunable: creates an editable field on your dashboard under:
    # FROGSubsystems/FlywheelSubsystem/tunables/target_speed
    @FROGSubsystem.tunable(default=0.0)
    def target_speed(self, speed: float):
        self.motor.set_velocity(speed)
```

---

## 4. Automated Motor Tuning Quickstart

Characterizing a mechanism to find its feedforward ($k_S, k_V, k_A$) and feedback ($k_P, k_D$) gains takes just a few lines:

```python
from froglib.subsystem import FROGSubsystem
from froglib.ctre import FROGTalonFX, CharacterizationConfig

class ElevatorSubsystem(FROGSubsystem):
    def __init__(self):
        super().__init__()
        self.motor = FROGTalonFX(
            id=20,
            signal_profile=FROGTalonFX.SignalProfile.POSITION_MM,
        )
        self.motor.config.feedback.sensor_to_mechanism_ratio = 12.5
        self.motor.apply_config()

    def get_characterization_command(self):
        """Creates a Command to execute autonomous characterization."""
        return self.motor.create_characterization_command(
            config=CharacterizationConfig(
                mechanism_name="Elevator",
                is_bounded=True,               # Protect limited travel
                min_test_pos=1.0,              # Minimum test rotation
                max_test_pos=18.0,             # Maximum test rotation
                safe_home_pos=5.0,             # Home position between test steps
                stator_current_limit_amps=18.0,# Current cap during test
                target_bandwidth_rad_s=20.0,   # Controller responsiveness
                auto_apply=True,               # Hot-apply gains directly to Slot 0
            )
        )
```

### Binding to Dashboard in `RobotContainer.py`

In `RobotContainer.py`, bind the characterization command to SmartDashboard or a controller trigger:

```python
from wpilib import SmartDashboard

class RobotContainer:
    def __init__(self):
        self.elevator = ElevatorSubsystem()

        # Expose button in Glass, Shuffleboard, or AdvantageScope
        SmartDashboard.putData(
            "Characterize Elevator",
            self.elevator.get_characterization_command(),
        )
```

When triggered, the characterizer executes its automated test routine, prints an identification report to the console, publishes live telemetry to `FROGLib/Tuning/Elevator`, and hot-applies the optimal gains directly to the motor's firmware.

---

## Next Steps

- Consult the [TalonFX Gain Tuning & Characterization Guide](tuning_guide.md) for deep dives on feedforward mathematics, pole placement, and SysId integration.
- Read [Testing & Verifying Automated Motor Tuning](testing_tuning.md) to learn how to test mechanisms with unit tests and WPILib simulation before deploying to hardware.
- Browse the **API Reference** in the navigation menu for detailed class and function specifications.
