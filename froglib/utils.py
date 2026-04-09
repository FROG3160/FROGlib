from itertools import chain, islice
from collections import deque
import math
from wpimath.geometry import (
    Transform3d,
    Translation3d,
    Rotation2d,
    Rotation3d,
    Pose3d,
    Pose2d,
    Transform2d,
)
from statistics import stdev


def constrain_radians(rads):
    """Returns radians between -pi and pi
    Args:
        rads (float): angle in radians"""
    return math.atan2(math.sin(rads), math.cos(rads))


def remap(val, OldMin, OldMax, NewMin, NewMax):
    """take a value in the old range and return a value in the new range"""
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def getRangeFromTransform(transform: Transform3d) -> float:
    translation = transform.translation()
    return math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)


def getAngleFromTransform(transform: Transform3d) -> float:
    translation = transform.translation()
    return math.atan2(translation.y, translation.x)


class RobotRelativeTarget:
    def __init__(self, robotPose: Pose2d, targetPose: Pose3d):
        # the docstring isn't really correct, but I don't have the time to wordsmith it
        # it gets x and y distances and uses those to calculate a robot-relative angle
        """Takes BlueAlliance-oriented robot and target poses and calculates field-oriented values for each alliance.

        Args:
            robotPose (Pose3d): The Blue Alliance robot pose
            targetPose (Pose3d): _description_

        """
        self.toTagFromRobot = targetPose.toPose2d() - robotPose
        self._x = self.toTagFromRobot.x
        self._y = self.toTagFromRobot.y
        self._z = targetPose.z
        self._heading = Rotation2d(self._x, self._y)
        self._flippedHeading = self._heading.rotateBy(Rotation2d(math.pi))

        self.driveHeading = self._heading
        self.firingHeading = self._flippedHeading
        self.fieldX = self._x
        self.fieldY = self._y
        self.fieldZ = self._z

        self.distance = math.sqrt(self.fieldX**2 + self.fieldY**2)
        # the length to the target including height... 3 dimensions
        self.range = math.sqrt(self.fieldX**2 + self.fieldY**2 + self.fieldZ**2)
        self.elevation = Rotation2d(math.acos(self.distance / self.range))
        self.driveVT = self.firingHeading.degrees() / 90


def partitionArray(array, indices):
    """_summary_

    Args:
        array (list): the array of vaules
        indices (list): a list of relative indexes.  e.g. [3, 2, 5] would return
            a list with the array broken up into the first 3 elements, then the
            next 2 elements, then 5 elements.
    Returns:
        list : a list of lists broken up by the given indices
    """
    i = iter(array)
    return [list(islice(i, n)) for n in chain(indices, [None])]


def arrayToPose3d(array):
    return Pose3d(
        array[0],
        array[1],
        array[2],
        Rotation3d.fromDegrees(array[3], array[4], array[5]),
    )


class Buffer(deque):
    def __init__(self, size: int, validLength: int = 1):
        """Constructor for Buffer
        Args:
            size (int): Maximum size of the buffer.  The largest number of values
                the buffer will keep.
            validLength (int, optional): The number of values in the buffer needed
                to treat the amount of data as valid. average() returns None if
                there aren't enough values.  Defaults to 1.
        """
        self.validLength = validLength
        super().__init__(maxlen=size)

    def _filterList(self):
        # our calculations can't accept None values
        return [x for x in self if x is not None]

    def _getBufferLength(self):
        return len(self._filterList())

    def _isValidData(self):
        return self._getBufferLength() >= self.validLength

    def average(self) -> float:
        """Get the average of all values in the buffer.
        Returns:
            float: The average of all values in the buffer if the number of values
                is >= the validLength parameter.
            None:  Returned if there aren't enough values to be >= the validLength
                parameter.
        """
        if self._isValidData():
            filteredList = self._filterList()
            return sum(filteredList) / len(filteredList)
        else:
            return None


class PoseBuffer(Buffer):
    def x_average(self):
        if self._isValidData():
            return sum([x.X() for x in self._filterList()]) / len(self._filterList())

    def y_average(self):
        if self._isValidData():
            return sum([x.Y() for x in self._filterList()]) / len(self._filterList())

    def rotation_average(self):
        if self._isValidData():
            return Rotation2d(
                sum([x.rotation().radians() for x in self._filterList()])
                / len(self._filterList())
            ).radians()

    def pose_average(self):
        return Pose2d(self.x_average(), self.y_average(), self.rotation_average())

    def x_stddev(self):
        if self._getBufferLength() > 2:
            return stdev([x.X() for x in self._filterList()])
        else:
            return 0.0

    def y_stddev(self):
        if self._getBufferLength() > 2:
            return stdev([x.Y() for x in self._filterList()])
        else:
            return 0.0

    def rotation_stddev(self):
        if self._getBufferLength() > 2:
            return stdev([x.rotation().radians() for x in self._filterList()])
        else:
            return 0.0


class GearStage:
    def __init__(self, input_gear, output_gear):
        self.input_gear = input_gear
        self.output_gear = output_gear

    def get_reduction(self):
        return self.output_gear / self.input_gear


class DriveTrain:
    def __init__(
        self, gear_stages: list = [GearStage(1, 1)], wheel_diameter: float = 1.0
    ):
        """Constructs a DriveTrain object that stores data about the gear stages and wheel.

        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            wheel_diameter (float): Diameter of the attached wheel in meters
        """
        self.gear_stages = gear_stages
        self.gear_reduction = math.prod(
            [stage.get_reduction() for stage in self.gear_stages]
        )
        self.circumference = math.pi * wheel_diameter
        self.system_reduction = self.gear_reduction / self.circumference
        self.rotation_to_distance_factor = self.circumference / self.gear_reduction

    def input_rotations(self, output_rotations):
        """Calculates motor rotations given the rotation at the other end of the gears."""
        return output_rotations * self.gear_reduction

    def output_rotations(self, input_rotations):
        """Calculates final gear rotation given the motor's rotation"""
        return input_rotations / self.gear_reduction

    def speed_to_input_rps(self, speed: float) -> float:
        """Converts the system linear speed to a motor velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: motor rotations per second
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.input_rotations(wheel_rotations_sec)
        return motor_rotations_sec

    def input_rps_to_speed(self, rotations_per_sec: float) -> float:
        """Converts motor velocity to the system linear speed

        Args:
            rotations_per_sec (float): motor rotational speed in rotations per second
        Returns:
            float: system linear speed in meters per second
        """
        wheel_rotations_sec = self.output_rotations(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def rotations_to_distance(self, rotations: float) -> float:
        """Takes  and returns distance

        Args:
            rotations (float): number of motor rotations
        Returns:
            float: distance in meters
        """
        wheel_rotations = self.output_rotations(rotations)
        return wheel_rotations * self.circumference

