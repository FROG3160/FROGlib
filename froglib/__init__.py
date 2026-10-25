"""FROGlib: Hardware Abstraction and Utility Library for FROG 3160."""

from froglib.ctre import (
    CharacterizationConfig,
    FROGCanCoder,
    FROGPigeonGyro,
    FROGSysIdHelper,
    FROGTalonFX,
    TalonFXCharacterizer,
    TuningResult,
    calculate_feedback_gains,
    linear_regression,
)

__all__ = [
    "CharacterizationConfig",
    "FROGCanCoder",
    "FROGPigeonGyro",
    "FROGSysIdHelper",
    "FROGTalonFX",
    "TalonFXCharacterizer",
    "TuningResult",
    "calculate_feedback_gains",
    "linear_regression",
]
