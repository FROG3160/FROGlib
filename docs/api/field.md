# Field API

## Overview

The `froglib.field` module simplifies working with official FRC AprilTag field layouts in RobotPy.

AprilTags are visual fiducial markers placed in standardized locations around the FRC field. In vision pipelines and localization routines, robots compare detected AprilTag transforms with known 3D field coordinates to determine their exact position on the field. `FROGField` wraps `AprilTagFieldLayout` to load official game layouts and query tag locations cleanly.

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| [`FROGField`][froglib.field.FROGField] | Class | Wrapper for `AprilTagFieldLayout` that loads official FRC field layouts and queries 3D tag poses. |

## Usage Examples

### Loading a Field Layout and Querying Tag Poses

```python
from robotpy_apriltag import AprilTagField
from froglib.field import FROGField

# 1. Load the official game layout for the 2024 season
field = FROGField(AprilTagField.k2024Crescendo)

# 2. Query the 3D pose of a specific AprilTag (e.g., Tag 7 - Blue Speaker Center)
tag_7_pose = field.getTagPose(7)

if tag_7_pose is not None:
    translation = tag_7_pose.translation()
    rotation = tag_7_pose.rotation()
    print(f"Tag 7 position: X={translation.x:.2f}m, Y={translation.y:.2f}m, Z={translation.z:.2f}m")
    print(f"Tag 7 rotation (yaw): {rotation.z_degrees:.1f}°")
else:
    print("Tag ID not found in field layout")
```

## Reference

::: froglib.field
