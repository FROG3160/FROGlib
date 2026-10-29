"""Field layout and AprilTag pose management for FIRST Robotics Competition.

This module provides abstractions for loading official FRC field layouts and
querying known 3D AprilTag poses from `robotpy_apriltag`.
"""

import math
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class FROGField:
    """Manager for FRC field AprilTag layout and fiducial positions.

    Wraps `robotpy_apriltag.AprilTagFieldLayout` to load official game field
    definitions and query tag poses by their fiducial ID.

    Attributes:
        _layout (AprilTagFieldLayout): Loaded AprilTag field layout instance.
    """

    def __init__(self, field_layout: AprilTagField):
        """Loads the specified AprilTag field layout.

        Args:
            field_layout (AprilTagField): Enum identifier for the official game field
                (e.g., AprilTagField.k2024Crescendo).
        """
        self._layout = AprilTagFieldLayout().loadField(field_layout)

    def getTagPose(self, tag_id: int):
        """Retrieves the 3D pose of an AprilTag on the field.

        Args:
            tag_id (int): Fiducial tag ID number.

        Returns:
            Pose3d | None: The 3D pose of the tag relative to the field origin,
                or None if the tag is not present in the layout.
        """
        return self._layout.getTagPose(tag_id)
