## Swerve Module Constants
from wpimath.units import inchesToMeters
from FROGlib.utils import GearStage


MK4C_L3_GEARING = [
    GearStage(16, 50),
    GearStage(28, 16),
    GearStage(15, 45),
]  # Mk4c L3
MK5I_R1_GEARING = [
    GearStage(12, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
MK5I_R2_GEARING = [
    GearStage(14, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
MK5I_R3_GEARING = [
    GearStage(16, 54),
    GearStage(32, 25),
    GearStage(15, 30),
]
WHEEL_DIAMETER = inchesToMeters(4)  # The tread has worn down
