"""FROGlib: Hardware Abstraction and Utility Library for FROG 3160."""

from froglib.ctre import (
    MAX_FALCON_RPM,
    MAX_KRAKEN_X60_RPM,
    CharacterizationConfig,
    FROGCanCoder,
    FROGCANCoderConfig,
    FROGFeedbackConfig,
    FROGPigeonGyro,
    FROGSysIdHelper,
    FROGTalonFX,
    FROGTalonFXConfig,
    TalonFXCharacterizer,
    TuningResult,
    calculate_feedback_gains,
    get_frog_cancoder_config,
    get_frog_talon_config,
    linear_regression,
)

__all__ = [
    "CharacterizationConfig",
    "FROGCanCoder",
    "FROGCANCoderConfig",
    "FROGFeedbackConfig",
    "FROGPigeonGyro",
    "FROGSysIdHelper",
    "FROGTalonFX",
    "FROGTalonFXConfig",
    "MAX_FALCON_RPM",
    "MAX_KRAKEN_X60_RPM",
    "TalonFXCharacterizer",
    "TuningResult",
    "calculate_feedback_gains",
    "get_frog_cancoder_config",
    "get_frog_talon_config",
    "linear_regression",
]
