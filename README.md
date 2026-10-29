# FROGlib

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![RobotPy 2026+](https://img.shields.io/badge/RobotPy-2026+-brightgreen.svg)](https://robotpy.github.io/)
[![Phoenix 6](https://img.shields.io/badge/CTRE_Phoenix_6-26.0+-orange.svg)](https://v6.docs.ctr-electronics.com/)
[![License](https://img.shields.io/badge/license-MIT-informational.svg)](LICENSE)

**FROGlib** is a high-level hardware abstraction and utility library developed for **FIRST Robotics Competition (FRC) Team 3160 — the FROGs**. Built on top of **RobotPy**, **CTRE Phoenix 6**, and **WPILib**, FROGlib accelerates robot software development by providing robust hardware wrappers, declarative subsystem telemetry/tunables, advanced swerve kinematics, multi-tag AprilTag vision pose estimation, and automated on-robot motor characterization and gain tuning.

---

## Repository Status & Release Tracks

FROGlib maintains two active release tracks:

| Release | Status | Focus / Description |
| :--- | :--- | :--- |
| **`2026.1.2`** | **Latest Stable 2026** | Verified for 2026 competition season; includes swerve drive, vision, subsystem framework, and core CTRE Phoenix 6 hardware wrappers. |
| **`2027.0.0a1`** | **2027 Alpha Pre-Release** | Cutting-edge track featuring automated CTRE TalonFX motor characterization (`TalonFXCharacterizer`), WPILib SysIdRoutine integration (`FROGSysIdHelper`), and physics simulation plants. |

### Installing via pip

You can install either release track directly from GitHub using `pip`:

```bash
# Install the stable 2026 release
pip install git+https://github.com/FROG3160/FROGlib.git@2026.1.2

# Install the 2027 alpha pre-release
pip install git+https://github.com/FROG3160/FROGlib.git@2027.0.0a1
```

### Specifying Dependencies in `pyproject.toml`

Add FROGlib to your robot project's dependencies:

```toml
[project]
dependencies = [
    # Pin to 2026 stable release
    "froglib @ git+https://github.com/FROG3160/FROGlib.git@2026.1.2",
    
    # OR pin to 2027 alpha pre-release
    # "froglib @ git+https://github.com/FROG3160/FROGlib.git@2027.0.0a1",
]
```

### Local Editable Installation (Development)

For local development or testing modifications directly inside a robot project workspace:

```bash
git clone https://github.com/FROG3160/FROGlib.git
cd FROGlib
pip install -e .
```

---

## Key Features

### 1. Automated CTRE Motor Tuning & Characterization (`froglib.ctre`)
- **`TalonFXCharacterizer`**: Autonomous on-robot state machine that determines $k_S$ (stiction), $k_V$ (velocity feedforward), and $k_A$ (acceleration feedforward) in mechanism units directly on the robot or in simulation.
- **Feedback Gain Synthesis**: Employs continuous-time 2nd-order pole placement ($k_P = k_A \cdot \omega_n^2$, $k_D = 2 \zeta \omega_n k_A - k_V$) to synthesize optimal position/velocity gains with natural back-EMF damping compensation.
- **Safety & Boundary Protection**: Travel envelope clamping (`min_test_pos`, `max_test_pos`, `safe_home_pos`), stator current clamping (`stator_current_limit_amps`), stall detection, and emergency aborts protect elevators, arms, and wrists.
- **Gearing Cross-Validation**: Validates measured $k_V$ against theoretical free speed to detect inverted or missing `sensor_to_mechanism_ratio` configurations.
- **`FROGSysIdHelper`**: Seamless bridge to WPILib `SysIdRoutine` with CTRE Phoenix 6 `SignalLogger` recording high-frequency 250 Hz `.hoot` logs for desktop SysId analysis.
- **Configuration & Results**: `CharacterizationConfig` dataclass and `TuningResult` with automatic export to `Slot0Configs`/`Slot1Configs` and Python code generation.

### 2. CTRE Phoenix 6 Hardware Wrappers & Configuration (`froglib.ctre`)
All CTRE Phoenix 6 hardware wrappers and configuration classes (`FROGTalonFX`, `FROGTalonFXConfig`, `FROGFeedbackConfig`, `FROGCanCoder`, `FROGCANCoderConfig`, `FROGPigeonGyro`, `TalonFXCharacterizer`, `FROGSysIdHelper`, `get_frog_talon_config`, `get_frog_cancoder_config`) are consolidated into `froglib.ctre`:
- **`FROGTalonFX` & `FROGTalonFXConfig`**: Enhanced Falcon 500 and Kraken X60 motor controller wrapper with feedback (`FROGFeedbackConfig`) and gain slot helpers, alongside:
  - `SignalProfile` CAN bus frame rate optimization (`BASIC`, `FLYWHEEL`, `POSITION_MM`, `SWERVE_DRIVE`, `SWERVE_AZIMUTH`, `TUNING`) reducing CAN utilization from >85% down to <45%.
  - Built-in `simulation_init()` and `simulation_update()` using WPILib `DCMotorSim` and `LinearSystemId`.
  - Factory helper `get_frog_talon_config()`.
- **`FROGPigeonGyro`**: Wrapper for Pigeon 2.0 IMU with yaw/pitch/roll retrieval and zeroing.
- **`FROGCanCoder` & `FROGCANCoderConfig`**: Magnetic absolute encoder wrapper with sensor-to-mechanism ratio synchronization, steering offset handling, and factory helper `get_frog_cancoder_config()`.

### 3. Declarative Subsystem Architecture (`froglib.subsystem`)
- **`FROGSubsystem`**: Base class extending WPILib `Subsystem` with declarative descriptors.
- **`@telemetry`**: Automatically publishes method return values to NetworkTables 4 and streams data to WPILib `DataLogManager` (.wpilog).
- **`@tunable`**: Exposes live parameters editable directly from Glass, Shuffleboard, or AdvantageScope dashboards with automatic setter callbacks.

### 4. Swerve Drive Kinematics & Pose Estimation (`froglib.swerve`, `froglib.sds`)
- **`SwerveChassis`**: Multi-module chassis coordinator handling field-oriented drive, robot-oriented drive, rotation PID heading lock (`RotationControllerConfig`), and odometry.
- **`SwerveModule`**: Coordinates drive and azimuth motor control with continuous absolute steering alignment via CANcoder.
- **SDS Presets (`froglib.sds`)**: Verified gear ratio definitions for Swerve Drive Specialties modules:
  - MK4c L3 (`MK4C_L3_GEARING`)
  - MK5i R1, R2, R3 (`MK5I_R1_GEARING`, `MK5I_R2_GEARING`, `MK5I_R3_GEARING`)

### 5. Vision & Object Detection (`froglib.vision`)
- **`FROGPoseEstimator`**: Multi-tag AprilTag pose estimator combining PhotonVision cameras and WPILib `AprilTagFieldLayout` with dynamic standard-deviation filtering.
- **`FROGDetector`**: Object detector built on `PhotonCamera` with custom numpy-based spatial clustering to identify and filter game piece targets.

### 6. Controllers, Sensors & Math Utilities (`froglib.xbox`, `froglib.navx`, `froglib.field`, `froglib.utils`)
- **`FROGXboxDriver` & `FROGXboxTactical`**: Driver and tactical operator controllers with stick shaping, exponential curves, deadbands, debounced triggers, and rumble feedback.
- **`FROGNavXGyro` (alias `FROGGyro`)**: Kauai Labs NavX IMU wrapper in `froglib.navx` with alliance-aware resets and `getAngleConstrained()`.
- **`FROGField`**: Coordinate transforms between red/blue alliances and field points of interest.
- **Utilities (`froglib.utils`)**: Mechanical drivetrain calculations (`DriveTrain`, `GearStage`), circular data buffers (`Buffer`, `PoseBuffer`), and `RobotRelativeTarget` coordinate helpers.

---

## Testing & Physics Simulation

FROGlib includes a comprehensive unit test and physics simulation suite verifying algorithmic correctness, state machine safety, and fault injection:

```bash
# Run all tests via unittest
python3 -m unittest discover -s tests -v

# Run the characterization and tuning test suite specifically
python3 -m unittest tests/test_tuning.py -v
```

The test suite runs against `SimulatedTalonFXPlant`, which simulates the continuous motor differential equation:
$$V = k_S \cdot \text{sign}(\omega) + k_V \cdot \omega + k_A \cdot \alpha$$
including static stiction, back-EMF, mechanism inertia, stator current limiting, and motor jam fault injection.

---

## Releasing a New Version

FROGlib uses dynamic versioning powered by **`setuptools_scm`**. Package version numbers are derived automatically from Git tags rather than manual version strings in `pyproject.toml`.

### Tagging and Releasing

1. Ensure all tests pass cleanly:
   ```bash
   python3 -m unittest discover -s tests -v
   ```

2. Create an annotated Git tag following semantic versioning (e.g., `2027.0.0a2` or `2026.1.3`):
   ```bash
   git tag -a 2027.0.0a2 -m "Release 2027.0.0a2: Characterization safety enhancements"
   ```

3. Push the tag to GitHub:
   ```bash
   git push origin 2027.0.0a2
   ```

4. Create a GitHub Release using the GitHub CLI:
   ```bash
   gh release create 2027.0.0a2 --generate-notes
   ```

### Building Distribution Wheels

To build distribution wheels for deployment on offline roboRIOs or team repositories:

```bash
pip install --upgrade build
python3 -m build
```

The built `.whl` and `.tar.gz` packages will be placed in the `dist/` directory.

---

## Documentation

Full documentation, API references, and guides are located in the `docs/` directory:
- [Getting Started Guide](docs/getting_started.md)
- [Tuning Guide](docs/tuning_guide.md)
- [Testing & Verifying Motor Tuning](docs/testing_tuning.md)

---

## License

FROGlib is open source and licensed under the [MIT License](LICENSE).
