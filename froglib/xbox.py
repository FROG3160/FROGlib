import math
from wpilib import XboxController, Timer
from commands2.button import CommandXboxController
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband
from wpilib.interfaces import GenericHID
import constants

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble


class FROGXboxDriver(CommandXboxController):
    """Custom Xbox Controller class for the driver controller specifically
    for field-oriented swerve drive control.
    """

    MODE = 0  # run auto routines

    def __init__(self, port, deadband, debouncePeriod, translationSlew, rotSlew):
        super().__init__(port)
        self.button_latest = {}
        self.timer = Timer()
        self.deadband = deadband
        self.debounce_period = debouncePeriod
        self.xSlew = SlewRateLimiter(translationSlew)
        self.ySlew = SlewRateLimiter(translationSlew)
        self.rotSlew = SlewRateLimiter(rotSlew)

    def getFieldHeading(self) -> float:
        """Get the desired robot heading from the Xbox's right
        stick.

        Returns:
            float: heading in radians with CCW positive
        """
        left = -self.getRightX()
        forward = -self.getRightY()
        # convert to radians
        return math.atan2(left, forward)

    def getFieldRotation(self) -> float:
        """Get the speed/rate of rotation from the Xbox's right
        stick X axis, with applied deadband.

        Returns:
            float: rotational speed factor from -1 to 1 with CCW being positive
        """
        return applyDeadband(-self.getRightX(), self.deadband)

    def getSlewLimitedFieldRotation(self) -> float:
        return self.rotSlew.calculate(self.getFieldRotation())

    def getFieldForward(self):
        return applyDeadband(-self.getLeftY(), self.deadband)

    def getSlewLimitedFieldForward(self):
        return self.xSlew.calculate(self.getFieldForward())

    def getFieldLeft(self):
        return applyDeadband(-self.getLeftX(), self.deadband)

    def getSlewLimitedFieldLeft(self):
        return self.ySlew.calculate(self.getFieldLeft())

    def getFieldThrottle(self):
        return applyDeadband(self.getRightTriggerAxis(), 0)

    def getPOVDebounced(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get("POV", 0)) > self.debounce_period:
                self.button_latest["POV"] = now
                val = pov
        if (now - self.button_latest.get("POV", 0)) < self.debounce_period:
            self.setRumble(RIGHT_RUMBLE, 1)
        else:
            self.setRumble(RIGHT_RUMBLE, 0)
        # self.update_nt("button_pov", val)
        return val
    
    def leftRumble(self):
        self._hid.setRumble(LEFT_RUMBLE, 1)

    def stopLeftRumble(self):
        self._hid.setRumble(LEFT_RUMBLE, 0)

    def rightRumble(self):
        self._hid.setRumble(RIGHT_RUMBLE, 1)

    def stopRightRumble(self):
        self._hid.setRumble(RIGHT_RUMBLE, 0)


class FROGXboxOperator(CommandXboxController):
    """Custom Xbox Controller class for the operator controller"""

    def __init__(self, port, deadband):
        super().__init__(port)
        self.deadband = deadband

    def getIntakeWheelSpeed(self):
        return applyDeadband(self.getLeftY(), self.deadband)

    def getTransferWheelSpeed(self):
        return applyDeadband(self.getRightY(), self.deadband)

    def getLeadscrewPosition(self):
        return self.getLeftTriggerAxis()

    def getFlyWheelSpeed(self):
        return self.getRightTriggerAxis()
