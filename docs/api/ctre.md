# CTRE Phoenix 6 API

The `froglib.ctre` module provides hardware abstraction wrappers for CTRE Phoenix 6 devices, frame-aligned CAN bus utilization optimization, automated empirical system identification, and WPILib simulation integration.

---

## Overview

Modern FRC robots commonly employ dozens of CAN devices (Kraken X60, Falcon 500, CANcoder, Pigeon 2.0). By default, high-frequency status signals can quickly saturate the CAN bus, resulting in dropped frames, latency spikes, and degraded closed-loop control.

!!! note "Consolidated CTRE Architecture"
    All CTRE Phoenix 6 hardware wrappers and configuration classes (`FROGTalonFX`, `FROGTalonFXConfig`, `FROGFeedbackConfig`, `FROGCanCoder`, `FROGCANCoderConfig`, `FROGPigeonGyro`, `TalonFXCharacterizer`, `FROGSysIdHelper`, `get_frog_talon_config`, `get_frog_cancoder_config`) from the former `motors` and `sensors` modules are now unified in `froglib.ctre`.

`froglib.ctre` provides:

1. **CAN Bus Optimization (`FROGTalonFX.SignalProfile`)**: Groups telemetry into three update frames—*Primary* (motion data), *Secondary* (power data), and *Health* (diagnostics/faults). Preconfigured profiles configure update rates tailored to mechanism roles (e.g., swerve drive, azimuth steering, flywheel, follower), slashing CAN bus utilization from over 80% to under 35%.
2. **Standard Configurations**: Predefined, immutable configuration objects for motor output neutral modes, inversion states, and CANcoder magnetic sensor orientations.
3. **Automated Characterization (`TalonFXCharacterizer`)**: Built-in state machine that automatically runs breakaway friction ($k_S$), steady-state velocity sweeps ($k_V$), and acceleration pulses ($k_A$). It then computes critically damped proportional ($k_P$) and derivative ($k_D$) gains using continuous pole placement.
4. **SysId & SignalLogger Integration (`FROGSysIdHelper`)**: Coordinates WPILib's `SysIdRoutine` with Phoenix 6 `SignalLogger`, temporarily shifting motor telemetry to 250 Hz during testing and restoring production profiles upon completion.
5. **Physics Simulation**: Seamless simulation using WPILib's `LinearSystemId` and `DCMotorSim` for high-fidelity mechanism modeling in desktop testing.

---

## Key Constants

| Constant | Type | Description |
| :--- | :--- | :--- |
| `MOTOR_OUTPUT_CWP_BRAKE` | `MotorOutputConfigs` | Motor output configured for **Brake** neutral mode and **Clockwise Positive** rotation. |
| `MOTOR_OUTPUT_CCWP_BRAKE` | `MotorOutputConfigs` | Motor output configured for **Brake** neutral mode and **Counter-Clockwise Positive** rotation. |
| `MOTOR_OUTPUT_CWP_COAST` | `MotorOutputConfigs` | Motor output configured for **Coast** neutral mode and **Clockwise Positive** rotation. |
| `MOTOR_OUTPUT_CCWP_COAST` | `MotorOutputConfigs` | Motor output configured for **Coast** neutral mode and **Counter-Clockwise Positive** rotation. |
| `MAGNET_SENSOR_CCWP` | `MagnetSensorConfigs` | CANcoder magnet sensor configured with **Counter-Clockwise Positive** direction and $[0, 1)$ absolute range. |
| `MAGNET_CONFIG_CONTWRAP_CCWP` | `MagnetSensorConfigs` | Backward-compatibility alias for `MAGNET_SENSOR_CCWP`. |
| `MAX_FALCON_RPM` | `float` | Theoretical maximum free speed for CTRE Falcon 500 motors (6380.0 RPM). |
| `MAX_KRAKEN_X60_RPM` | `float` | Theoretical maximum free speed for CTRE Kraken X60 motors (6000.0 RPM). |

---

## Key Components

| Component | Type | Description |
| :--- | :--- | :--- |
| [`FROGTalonFX`][froglib.ctre.FROGTalonFX] | `class (TalonFX)` | Primary motor abstraction with `SignalProfile` optimization, stall detection, automated tuning, and simulation. |
| `FROGTalonFX.SignalProfile` | `Enum` | Frame rate profiles `(primary, secondary, health)` in Hz: `SWERVE_DRIVE`, `SWERVE_AZIMUTH`, `FLYWHEEL`, `POSITION_MM`, `FOLLOWER`, `BASIC`, `TUNING`, `DISABLED`. |
| [`FROGTalonFXConfig`][froglib.ctre.FROGTalonFXConfig] | `class (TalonFXConfiguration)` | Convenience motor configuration class accepting feedback configs and Slot 0 / Slot 1 gains in its constructor (unified from legacy `motors`). |
| [`FROGFeedbackConfig`][froglib.ctre.FROGFeedbackConfig] | `class (FeedbackConfigs)` | Feedback configuration helper allowing inline sensor source and remote sensor CAN ID assignment (unified from legacy `motors`). |
| [`FROGCanCoder`][froglib.ctre.FROGCanCoder] | `class (CANcoder)` | CANcoder absolute encoder initialized with `MAGNET_SENSOR_CCWP` and optimized bus utilization. |
| [`FROGCANCoderConfig`][froglib.ctre.FROGCANCoderConfig] | `class (CANcoderConfiguration)` | CANcoder configuration class with continuous wraparound, CCW-positive sensor direction, and steer magnet offset (unified from legacy `sensors`). |
| [`FROGPigeonGyro`][froglib.ctre.FROGPigeonGyro] | `class (Pigeon2)` | Pigeon 2.0 gyro wrapper configured with 100 Hz yaw updates and bus optimization. |
| [`TalonFXCharacterizer`][froglib.ctre.TalonFXCharacterizer] | `class` | Autonomous state machine measuring empirical feedforward ($k_S, k_V, k_A$) and calculating feedback gains ($k_P, k_D$). |
| [`CharacterizationConfig`][froglib.ctre.CharacterizationConfig] | `dataclass` | Safety limits, test voltages, travel bounds, and pole-placement bandwidth settings for `TalonFXCharacterizer`. |
| [`TuningResult`][froglib.ctre.TuningResult] | `dataclass` | Structured characterization results with converters to `Slot0Configs`, `Slot1Configs`, and python code snippets. |
| [`FROGSysIdHelper`][froglib.ctre.FROGSysIdHelper] | `class` | Integrates WPILib `SysIdRoutine` with CTRE `SignalLogger` and automatic signal frequency switching. |
| `get_frog_talon_config()` | `function` | Factory function returning a default `TalonFXConfiguration`. |
| `get_frog_cancoder_config(steer_offset=0.0)` | `function` | Factory function returning a `CANcoderConfiguration` with `MAGNET_SENSOR_CCWP` and optional steer magnet offset. |
| `linear_regression()` | `function` | Ordinary least-squares regression returning `(slope, intercept, r_squared)`. |
| `calculate_feedback_gains()` | `function` | Calculates critically damped $(k_P, k_D)$ gains via second-order continuous pole placement. |

---

## Usage Examples

### Instantiating a TalonFX with Signal Profiles

```python
from froglib.ctre import (
    FROGTalonFX,
    MOTOR_OUTPUT_CCWP_BRAKE,
    get_frog_talon_config,
)

config = get_frog_talon_config()
config.motor_output = MOTOR_OUTPUT_CCWP_BRAKE
config.current_limits.stator_current_limit_enable = True
config.current_limits.stator_current_limit = 40.0

# Initialize motor with SWERVE_DRIVE signal profile (50Hz motion, 50Hz electrical, 4Hz health)
drive_motor = FROGTalonFX(
    id=1,
    motor_config=config,
    canbus="rio",
    motor_name="FrontLeftDrive",
    signal_profile=FROGTalonFX.SignalProfile.SWERVE_DRIVE,
)
```

### Running Automated Mechanism Characterization

```python
from froglib.ctre import FROGTalonFX, CharacterizationConfig

motor = FROGTalonFX(id=5, motor_name="ShooterFlywheel")

# Configure automated test
test_cfg = CharacterizationConfig(
    mechanism_name="ShooterFlywheel",
    is_bounded=False,
    max_voltage=9.0,
    target_bandwidth_rad_s=30.0,
    auto_apply=True,  # Hot-apply gains directly to Slot 0
    target_slot=0,
)

# Create runnable command for test mode
tuning_command = motor.create_characterization_command(test_cfg)
```

### Configuring WPILib SysIdRoutine with SignalLogger

```python
from froglib.ctre import FROGTalonFX

motor = FROGTalonFX(id=10, motor_name="Elevator")

# Create SysId helper with bounded travel limits
sysid_helper = motor.get_sysid_helper(
    mechanism_name="Elevator",
    ramp_rate_v_per_s=0.5,
    step_voltage=4.0,
    timeout_s=5.0,
    is_bounded=True,
    min_pos=0.1,
    max_pos=25.0,
)

# Bind SysId commands to joystick buttons in robot container
quasistatic_fwd_cmd = sysid_helper.quasistatic_forward()
dynamic_fwd_cmd = sysid_helper.dynamic_forward()
```

---

## Reference

::: froglib.ctre
