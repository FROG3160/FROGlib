"""Swerve Drive Specialties (SDS) hardware constants and gear ratios.

This module provides mechanical gear stage specifications and physical dimensions
for popular Swerve Drive Specialties (SDS) modules, specifically the MK4c and
MK5i (inverted) swerve modules. These constants are designed to be paired with
`froglib.utils.DriveTrain` or `froglib.swerve.SwerveModuleConfig` to configure
accurate odometry and drive kinematics.
"""

## Swerve Module Constants
from wpimath.units import inchesToMeters
from froglib.utils import GearStage


MK4C_L3_GEARING = [
    GearStage(16, 50),
    GearStage(28, 16),
    GearStage(15, 45),
]
"""list[GearStage]: Gear stages for the SDS MK4c Level 3 swerve module (overall drive reduction ~6.12:1)."""

MK5I_R1_GEARING = [
    GearStage(12, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
"""list[GearStage]: Gear stages for the SDS MK5i Ratio 1 swerve module (overall drive reduction ~8.14:1)."""

MK5I_R2_GEARING = [
    GearStage(14, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
"""list[GearStage]: Gear stages for the SDS MK5i Ratio 2 swerve module (overall drive reduction ~6.75:1)."""

MK5I_R3_GEARING = [
    GearStage(16, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
"""list[GearStage]: Gear stages for the SDS MK5i Ratio 3 swerve module (overall drive reduction ~5.91:1)."""

WHEEL_DIAMETER = inchesToMeters(4)  # The tread has worn down
"""float: Nominal wheel diameter in meters for standard 4-inch SDS swerve wheels (0.1016 m)."""

