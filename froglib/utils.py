"""General utilities, geometry helpers, statistical buffers, and drivetrain math.

This module provides common utilities for FRC robot code, including:
- Angle wrapping and range remapping functions.
- 3D transform metrics and robot-to-target geometry calculations (`RobotRelativeTarget`).
- Moving window statistical buffers (`Buffer`, `PoseBuffer`).
- Gear ratio and mechanical drivetrain modeling (`GearStage`, `DriveTrain`).
"""

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


def constrain_radians(rads: float) -> float:
    """Wraps an angle in radians to the range [-pi, pi].

    Args:
        rads (float): Angle in radians.

    Returns:
        float: Constrained angle in radians within [-pi, pi].
    """
    return math.atan2(math.sin(rads), math.cos(rads))


def remap(val: float, OldMin: float, OldMax: float, NewMin: float, NewMax: float) -> float:
    """Linearly remaps a value from an input range to a target range.

    Args:
        val (float): The value to remap.
        OldMin (float): Lower bound of the source range.
        OldMax (float): Upper bound of the source range.
        NewMin (float): Lower bound of the target range.
        NewMax (float): Upper bound of the target range.

    Returns:
        float: The mapped value within the target range.
    """
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def getRangeFromTransform(transform: Transform3d) -> float:
    """Calculates the 3D Euclidean distance of a 3D transform translation.

    Args:
        transform (Transform3d): The 3D transform.

    Returns:
        float: Straight-line distance in meters.
    """
    translation = transform.translation()
    return math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)


def getAngleFromTransform(transform: Transform3d) -> float:
    """Calculates the planar yaw angle of a 3D transform translation.

    Args:
        transform (Transform3d): The 3D transform.

    Returns:
        float: Planar yaw angle in radians.
    """
    translation = transform.translation()
    return math.atan2(translation.y, translation.x)


class RobotRelativeTarget:
    """Calculates geometric relationships between a robot pose and a target pose.

    Computes 2D distance, 3D Euclidean range, drive and firing headings, target elevation,
    and normalized steering angle offsets from the robot's current pose to a 3D target.

    Attributes:
        toTagFromRobot (Transform2d): 2D transform from the robot pose to the target pose.
        driveHeading (Rotation2d): Heading angle pointing directly toward the target.
        firingHeading (Rotation2d): Heading angle pointing directly away from the target (180° reversed).
        fieldX (float): Field-relative X delta from robot to target in meters.
        fieldY (float): Field-relative Y delta from robot to target in meters.
        fieldZ (float): Height of the target in meters.
        distance (float): 2D horizontal distance to target in meters.
        range (float): 3D straight-line distance to target in meters.
        elevation (Rotation2d): Elevation angle to target above the horizontal plane.
        driveVT (float): Virtual target steering factor normalized by 90 degrees.
    """

    def __init__(self, robotPose: Pose2d, targetPose: Pose3d):
        # the docstring isn't really correct, but I don't have the time to wordsmith it
        # it gets x and y distances and uses those to calculate a robot-relative angle
        """Calculates target heading, distances, and elevation from robot and target poses.

        Args:
            robotPose (Pose2d): Current 2D pose of the robot on the field.
            targetPose (Pose3d): Known 3D pose of the target on the field.
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


def partitionArray(array: list, indices: list) -> list:
    """Partitions a sequence into sublists according to consecutive slice lengths.

    Args:
        array (list): The list of values to partition.
        indices (list): A list of relative partition lengths. For example, `[3, 2, 5]`
            breaks the array into the first 3 elements, then the next 2 elements,
            and finally the remaining elements.

    Returns:
        list: A list of sublists broken up by the given slice lengths.
    """
    i = iter(array)
    return [list(islice(i, n)) for n in chain(indices, [None])]


def arrayToPose3d(array: list) -> Pose3d:
    """Constructs a Pose3d object from a 6-element list.

    Args:
        array (list): Sequence of 6 numbers `[x, y, z, roll, pitch, yaw]`,
            where translations are in meters and rotations are in degrees.

    Returns:
        Pose3d: The constructed 3D pose.
    """
    return Pose3d(
        array[0],
        array[1],
        array[2],
        Rotation3d.fromDegrees(array[3], array[4], array[5]),
    )


class Buffer(deque):
    """Rolling window buffer extending collections.deque with validation and averaging.

    Attributes:
        validLength (int): Minimum count of non-None values required for the buffer
            to be considered valid.
    """

    def __init__(self, size: int, validLength: int = 1):
        """Initializes the rolling buffer.

        Args:
            size (int): Maximum size of the buffer. The largest number of values
                the buffer will retain.
            validLength (int, optional): The number of values in the buffer needed
                to treat the data as valid. average() returns None if fewer values
                exist. Defaults to 1.
        """
        self.validLength = validLength
        super().__init__(maxlen=size)

    def _filterList(self):
        """Filters out None values from the buffer.

        Returns:
            list: List of non-None elements currently in the buffer.
        """
        # our calculations can't accept None values
        return [x for x in self if x is not None]

    def _getBufferLength(self):
        """Gets the count of non-None elements currently in the buffer.

        Returns:
            int: Number of valid elements.
        """
        return len(self._filterList())

    def _isValidData(self):
        """Checks if the buffer contains at least validLength elements.

        Returns:
            bool: True if sufficient valid data exists, False otherwise.
        """
        return self._getBufferLength() >= self.validLength

    def average(self) -> float | None:
        """Gets the average of all non-None values in the buffer.

        Returns:
            float | None: The arithmetic mean of all values if valid, or None if
                the buffer contains fewer items than validLength.
        """
        if self._isValidData():
            filteredList = self._filterList()
            return sum(filteredList) / len(filteredList)
        else:
            return None


class PoseBuffer(Buffer):
    """Rolling buffer of Pose2d objects for computing statistical pose metrics.

    Tracks historical robot poses to compute running averages and standard
    deviations across X, Y, and rotation coordinates.
    """

    def x_average(self) -> float | None:
        """Computes the running mean of the X translation coordinate.

        Returns:
            float | None: Average X value in meters, or None if buffer is not valid.
        """
        if self._isValidData():
            return sum([x.X() for x in self._filterList()]) / len(self._filterList())

    def y_average(self) -> float | None:
        """Computes the running mean of the Y translation coordinate.

        Returns:
            float | None: Average Y value in meters, or None if buffer is not valid.
        """
        if self._isValidData():
            return sum([x.Y() for x in self._filterList()]) / len(self._filterList())

    def rotation_average(self) -> float | None:
        """Computes the running mean of the rotation angle in radians.

        Returns:
            float | None: Average angle in radians, or None if buffer is not valid.
        """
        if self._isValidData():
            return Rotation2d(
                sum([x.rotation().radians() for x in self._filterList()])
                / len(self._filterList())
            ).radians()

    def pose_average(self) -> Pose2d:
        """Computes the average Pose2d across the buffer.

        Returns:
            Pose2d: Pose2d composed of the averaged X, Y, and rotation coordinates.
        """
        return Pose2d(self.x_average(), self.y_average(), self.rotation_average())

    def x_stddev(self) -> float:
        """Computes sample standard deviation of X translation.

        Returns:
            float: Standard deviation of X in meters (0.0 if fewer than 3 values).
        """
        if self._getBufferLength() > 2:
            return stdev([x.X() for x in self._filterList()])
        else:
            return 0.0

    def y_stddev(self) -> float:
        """Computes sample standard deviation of Y translation.

        Returns:
            float: Standard deviation of Y in meters (0.0 if fewer than 3 values).
        """
        if self._getBufferLength() > 2:
            return stdev([x.Y() for x in self._filterList()])
        else:
            return 0.0

    def rotation_stddev(self) -> float:
        """Computes sample standard deviation of rotation angle in radians.

        Returns:
            float: Standard deviation of rotation in radians (0.0 if fewer than 3 values).
        """
        if self._getBufferLength() > 2:
            return stdev([x.rotation().radians() for x in self._filterList()])
        else:
            return 0.0


class GearStage:
    """Represents a single reduction stage in a mechanical gear train.

    Attributes:
        input_gear (int | float): Driving gear tooth count.
        output_gear (int | float): Driven gear tooth count.
    """

    def __init__(self, input_gear: int | float, output_gear: int | float):
        """Initializes a gear stage.

        Args:
            input_gear (int | float): Driving gear tooth count.
            output_gear (int | float): Driven gear tooth count.
        """
        self.input_gear = input_gear
        self.output_gear = output_gear

    def get_reduction(self) -> float:
        """Calculates the gear reduction ratio (output / input).

        Returns:
            float: Gear reduction ratio.
        """
        return self.output_gear / self.input_gear


class DriveTrain:
    """Mechanical modeling and conversion utilities for robot drivetrains.

    Computes overall gear reductions, wheel circumference, and conversions
    between motor rotations/velocities and linear travel distance/speed.

    Attributes:
        gear_stages (list[GearStage]): List of sequential gear stages.
        gear_reduction (float): Overall total gear reduction ratio.
        circumference (float): Wheel circumference in meters.
        system_reduction (float): Overall reduction ratio divided by circumference.
        rotation_to_distance_factor (float): Meters traveled per motor rotation.
    """

    def __init__(
        self, gear_stages: list = [GearStage(1, 1)], wheel_diameter: float = 1.0
    ):
        """Constructs a DriveTrain object modeling gear stages and wheel parameters.

        Args:
            gear_stages (list, optional): List of GearStage objects. Defaults to [GearStage(1, 1)].
            wheel_diameter (float, optional): Diameter of the drive wheel in meters. Defaults to 1.0.
        """
        self.gear_stages = gear_stages
        self.gear_reduction = math.prod(
            [stage.get_reduction() for stage in self.gear_stages]
        )
        self.circumference = math.pi * wheel_diameter
        self.system_reduction = self.gear_reduction / self.circumference
        self.rotation_to_distance_factor = self.circumference / self.gear_reduction

    def input_rotations(self, output_rotations: float) -> float:
        """Calculates driving motor rotations given the output rotations at the wheel.

        Args:
            output_rotations (float): Rotations at the output wheel.

        Returns:
            float: Required motor rotations.
        """
        return output_rotations * self.gear_reduction

    def output_rotations(self, input_rotations: float) -> float:
        """Calculates final wheel rotations given the driving motor rotations.

        Args:
            input_rotations (float): Motor rotations.

        Returns:
            float: Resulting wheel rotations.
        """
        return input_rotations / self.gear_reduction

    def speed_to_input_rps(self, speed: float) -> float:
        """Converts desired robot linear speed to driving motor rotational velocity.

        Args:
            speed (float): Desired linear speed in meters per second.

        Returns:
            float: Motor rotational speed in rotations per second (RPS).
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.input_rotations(wheel_rotations_sec)
        return motor_rotations_sec

    def input_rps_to_speed(self, rotations_per_sec: float) -> float:
        """Converts driving motor rotational velocity to robot linear speed.

        Args:
            rotations_per_sec (float): Motor rotational speed in rotations per second (RPS).

        Returns:
            float: Robot linear speed in meters per second.
        """
        wheel_rotations_sec = self.output_rotations(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def rotations_to_distance(self, rotations: float) -> float:
        """Converts motor rotations to linear distance traveled by the wheel.

        Args:
            rotations (float): Number of motor rotations.

        Returns:
            float: Distance traveled in meters.
        """
        wheel_rotations = self.output_rotations(rotations)
        return wheel_rotations * self.circumference
