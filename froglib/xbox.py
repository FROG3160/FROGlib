import math
from wpilib import XboxController, Timer
from wpilib.event import EventLoop
from commands2.button import CommandXboxController, Trigger
from commands2 import CommandScheduler
from typing import Optional
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband
from wpilib.interfaces import GenericHID
from wpilib import DriverStation
import constants

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble


class FROGXboxDriver(CommandXboxController):
    """Custom Xbox Controller class for the driver controller specifically
    for field-oriented swerve drive control.
    """

    MODE = 0  # run auto routines
    RED_ALLIANCE = -1
    BLUE_ALLIANCE = 1

    def __init__(self, port, deadband, debouncePeriod, translationSlew, rotSlew):
        super().__init__(port)
        self.button_latest = {}
        self.timer = Timer()
        self.deadband = deadband
        self.debounce_period = debouncePeriod
        self.xSlew = SlewRateLimiter(translationSlew)
        self.ySlew = SlewRateLimiter(translationSlew)
        self.rotSlew = SlewRateLimiter(rotSlew)
        self.alliance = 0

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

    def getRotation(self) -> float:
        """Get the speed/rate of rotation from the Xbox's right
        stick X axis, with applied deadband.

        Returns:
            float: rotational speed factor from -1 to 1 with CCW being positive
        """
        return applyDeadband(-self.getRightX(), self.deadband)

    def getSlewLimitedFieldRotation(self) -> float:
        return self.rotSlew.calculate(self.getRotation())

    def getFieldForward(self):
        return applyDeadband(-self.getLeftY() * self.alliance, self.deadband)

    def getSlewLimitedFieldForward(self):
        return self.xSlew.calculate(self.getFieldForward())

    def getRobotForward(self):
        return applyDeadband(-self.getLeftY(), self.deadband)

    def getFieldLeft(self):
        return applyDeadband(-self.getLeftX() * self.alliance, self.deadband)

    def getSlewLimitedFieldLeft(self):
        return self.ySlew.calculate(self.getFieldLeft())

    def getRobotLeft(self):
        return applyDeadband(-self.getLeftX(), self.deadband)

    def getFieldThrottle(self):
        return applyDeadband(self.getRightTriggerAxis(), 0)

    def getPOVDebounced(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self._hid.getPOV()
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
        self.setRumble(LEFT_RUMBLE, 1)

    def stopLeftRumble(self):
        self.setRumble(LEFT_RUMBLE, 0)

    def rightRumble(self):
        self.setRumble(RIGHT_RUMBLE, 1)

    def stopRightRumble(self):
        self.setRumble(RIGHT_RUMBLE, 0)

    def set_alliance(self, alliance):
        if alliance == DriverStation.Alliance.kBlue:
            self.alliance = self.BLUE_ALLIANCE
        else:
            self.alliance = self.RED_ALLIANCE

    def pov0(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 0)

    def pov45(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 45)

    def pov90(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 90)

    def pov135(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 135)

    def pov180(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 180)

    def pov225(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 225)

    def pov270(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 270)

    def pov315(self, loop: Optional[EventLoop] = None) -> Trigger:
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 315)


class FROGXboxTactical(CommandXboxController):
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
