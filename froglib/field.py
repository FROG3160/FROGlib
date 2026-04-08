import math
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


class FROGField:
    def __init__(self, field_layout: AprilTagField):
        self._layout = AprilTagFieldLayout().loadField(field_layout)

    def getTagPose(self, tag_id: int):
        return self._layout.getTagPose(tag_id)
