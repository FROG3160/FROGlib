"""Xbox controller abstractions for driver and operator input in FRC.

This module provides customized subclasses of WPILib's `CommandXboxController`:
- `FROGXboxDriver`: Specialized for field-oriented swerve drive control, featuring
  stick deadband filtering, slew rate limiters, alliance-aware coordinate inversion,
  and debounced D-pad (POV) trigger bindings with haptic rumble feedback.
- `FROGXboxTactical`: Specialized for mechanism / tactical operator input with
  deadbanded axis getters for intake, transfer, leadscrew, and flywheel mechanisms.
"""

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

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble
LEFT_RUMBLE = GenericHID.RumbleType.kLeftRumble


class FROGXboxDriver(CommandXboxController):
    """Custom Xbox Controller for driver input in field-oriented swerve drive systems.

    Extends `CommandXboxController` with configurable deadbands, slew rate limiters
    on translation and rotation, alliance-aware forward/left direction flipping, and
    debounced D-pad (POV) buttons with tactile rumble feedback.

    Attributes:
        deadband (float): Deadband threshold below which stick inputs evaluate to 0.0.
        debounce_period (float): Minimum duration in seconds required between POV button triggers.
        xSlew (SlewRateLimiter): Slew rate limiter for forward/backward translation.
        ySlew (SlewRateLimiter): Slew rate limiter for left/right strafing translation.
        rotSlew (SlewRateLimiter): Slew rate limiter for angular rotation rate.
        alliance (int): Alliance scaling factor (1 for Blue Alliance, -1 for Red Alliance).
    """

    MODE = 0  # run auto routines
    RED_ALLIANCE = -1
    BLUE_ALLIANCE = 1

    def __init__(
        self,
        port: int,
        deadband: float,
        debouncePeriod: float,
        translationSlew: float,
        rotSlew: float,
    ):
        """Initializes the driver controller.

        Args:
            port (int): Driver Station USB port index for the controller.
            deadband (float): Joystick deadband value in range [0, 1].
            debouncePeriod (float): Debounce interval for POV buttons in seconds.
            translationSlew (float): Maximum rate of change of translation input units per second.
            rotSlew (float): Maximum rate of change of rotation input units per second.
        """
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
        """Gets the desired robot heading from the Xbox right stick.

        Returns:
            float: Desired heading in radians with CCW positive.
        """
        left = -self.getRightX()
        forward = -self.getRightY()
        # convert to radians
        return math.atan2(left, forward)

    def getRotation(self) -> float:
        """Gets the speed/rate of rotation from the Xbox right stick X axis with deadband.

        Returns:
            float: Rotational speed factor from -1 to 1 with CCW being positive.
        """
        return applyDeadband(-self.getRightX(), self.deadband)

    def getSlewLimitedFieldRotation(self) -> float:
        """Gets the slew-rate-limited rotational speed demand with applied deadband.

        Returns:
            float: Rate-limited rotational speed factor in range [-1, 1].
        """
        return self.rotSlew.calculate(self.getRotation())

    def getFieldForward(self) -> float:
        """Gets the alliance-oriented forward translation demand with applied deadband.

        Returns:
            float: Forward speed demand in range [-1, 1], inverted when on Red Alliance.
        """
        return applyDeadband(-self.getLeftY() * self.alliance, self.deadband)

    def getSlewLimitedFieldForward(self) -> float:
        """Gets the slew-rate-limited alliance-oriented forward translation demand.

        Returns:
            float: Slew-rate-limited forward speed demand in range [-1, 1].
        """
        return self.xSlew.calculate(self.getFieldForward())

    def getRobotForward(self) -> float:
        """Gets the robot-oriented forward translation demand with applied deadband.

        Returns:
            float: Robot-relative forward speed demand in range [-1, 1].
        """
        return applyDeadband(-self.getLeftY(), self.deadband)

    def getFieldLeft(self) -> float:
        """Gets the alliance-oriented left strafe translation demand with applied deadband.

        Returns:
            float: Leftward speed demand in range [-1, 1], inverted when on Red Alliance.
        """
        return applyDeadband(-self.getLeftX() * self.alliance, self.deadband)

    def getSlewLimitedFieldLeft(self) -> float:
        """Gets the slew-rate-limited alliance-oriented left strafe translation demand.

        Returns:
            float: Slew-rate-limited leftward speed demand in range [-1, 1].
        """
        return self.ySlew.calculate(self.getFieldLeft())

    def getRobotLeft(self) -> float:
        """Gets the robot-oriented left strafe translation demand with applied deadband.

        Returns:
            float: Robot-relative leftward speed demand in range [-1, 1].
        """
        return applyDeadband(-self.getLeftX(), self.deadband)

    def getFieldThrottle(self) -> float:
        """Gets the throttle demand from the right trigger axis.

        Returns:
            float: Right trigger axis value in range [0, 1].
        """
        return applyDeadband(self.getRightTriggerAxis(), 0)

    def getPOVDebounced(self) -> int:
        """Gets the debounced D-pad (POV) angle and triggers brief haptic rumble feedback.

        Returns:
            int: POV angle in degrees (0, 45, 90, etc.), or -1 if unpressed or bouncing.
        """
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
        """Activates full rumble on the left controller motor."""
        self.setRumble(LEFT_RUMBLE, 1)

    def stopLeftRumble(self):
        """Stops rumble on the left controller motor."""
        self.setRumble(LEFT_RUMBLE, 0)

    def rightRumble(self):
        """Activates full rumble on the right controller motor."""
        self.setRumble(RIGHT_RUMBLE, 1)

    def stopRightRumble(self):
        """Stops rumble on the right controller motor."""
        self.setRumble(RIGHT_RUMBLE, 0)

    def set_alliance(self, alliance):
        """Sets controller alliance mode to invert coordinate directions appropriately.

        Args:
            alliance (DriverStation.Alliance): Current DriverStation alliance color.
        """
        if alliance == DriverStation.Alliance.kBlue:
            self.alliance = self.BLUE_ALLIANCE
        else:
            self.alliance = self.RED_ALLIANCE

    def pov0(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 0° (up) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 0°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 0)

    def pov45(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 45° (up-right) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 45°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 45)

    def pov90(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 90° (right) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 90°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 90)

    def pov135(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 135° (down-right) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 135°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 135)

    def pov180(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 180° (down) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 180°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 180)

    def pov225(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 225° (down-left) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 225°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 225)

    def pov270(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 270° (left) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 270°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 270)

    def pov315(self, loop: Optional[EventLoop] = None) -> Trigger:
        """Creates a Command Trigger for the debounced 315° (up-left) POV button.

        Args:
            loop (EventLoop, optional): Event loop to bind the trigger to. Defaults to
                the CommandScheduler default button loop.

        Returns:
            Trigger: Command Trigger activated when POV button is at 315°.
        """
        if loop is None:
            loop = CommandScheduler.getInstance().getDefaultButtonLoop()
        return Trigger(loop, lambda: self.getPOVDebounced() == 315)


class FROGXboxTactical(CommandXboxController):
    """Custom Xbox Controller class for mechanism and operator controls.

    Extends `CommandXboxController` with deadbanded stick and trigger getters
    mapped to robot mechanism velocities and positions.

    Attributes:
        deadband (float): Deadband threshold applied to joystick axes.
    """

    def __init__(self, port: int, deadband: float):
        """Initializes the tactical/operator controller.

        Args:
            port (int): Driver Station USB port index.
            deadband (float): Joystick deadband value in range [0, 1].
        """
        super().__init__(port)
        self.deadband = deadband

    def getIntakeWheelSpeed(self) -> float:
        """Gets deadbanded speed for intake wheels from the left Y stick.

        Returns:
            float: Speed factor in range [-1, 1].
        """
        return applyDeadband(self.getLeftY(), self.deadband)

    def getTransferWheelSpeed(self) -> float:
        """Gets deadbanded speed for transfer mechanism wheels from the right Y stick.

        Returns:
            float: Speed factor in range [-1, 1].
        """
        return applyDeadband(self.getRightY(), self.deadband)

    def getLeadscrewPosition(self) -> float:
        """Gets position/command input for a leadscrew from the left trigger axis.

        Returns:
            float: Left trigger axis value in range [0, 1].
        """
        return self.getLeftTriggerAxis()

    def getFlyWheelSpeed(self) -> float:
        """Gets speed command for shooter flywheels from the right trigger axis.

        Returns:
            float: Right trigger axis value in range [0, 1].
        """
        return self.getRightTriggerAxis()
