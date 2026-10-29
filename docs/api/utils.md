# Utils API

## Overview

The `froglib.utils` module provides fundamental mathematical utilities, geometry calculations, statistical filtering buffers, and mechanical drivetrain modeling for FRC robots:

1. **Geometry and Targeting (`RobotRelativeTarget`, `constrain_radians`, `getRangeFromTransform`, `getAngleFromTransform`)**: Computes relative vectors, planar distances, 3D Euclidean distances, elevation pitch angles, and normalized virtual target steering variables (`driveVT`) between robot poses and field targets.
2. **Statistical Smoothing & Queues (`Buffer`, `PoseBuffer`)**: Rolling window ring-buffers with data validation thresholds. `PoseBuffer` tracks historical `Pose2d` measurements, computing running averages and standard deviations for X, Y, and rotation angle to filter noisy sensor and vision telemetry.
3. **Mechanical Drivetrain Math (`GearStage`, `DriveTrain`)**: Models multi-stage gearbox ratios and wheel circumferences to convert between motor rotations/velocity (RPS) and robot linear distance/speed (m/s).
4. **General Utilities (`remap`, `partitionArray`, `arrayToPose3d`)**: Utility helpers for range mapping, array partitioning, and constructing `Pose3d` objects from raw lists.

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| [`constrain_radians`][froglib.utils.constrain_radians] | Function | Constrains an angle in radians to the range $[-\pi, \pi]$. |
| [`remap`][froglib.utils.remap] | Function | Linearly scales a value from an initial range to a destination range. |
| [`getRangeFromTransform`][froglib.utils.getRangeFromTransform] | Function | Computes the 3D Euclidean straight-line distance of a `Transform3d` translation. |
| [`getAngleFromTransform`][froglib.utils.getAngleFromTransform] | Function | Computes the planar yaw angle in radians of a `Transform3d` translation. |
| [`RobotRelativeTarget`][froglib.utils.RobotRelativeTarget] | Class | Computes 2D distance, 3D range, drive/firing headings, elevation, and steering factor from robot pose to target pose. |
| [`partitionArray`][froglib.utils.partitionArray] | Function | Partitions a list into consecutive sublists by given slice lengths. |
| [`arrayToPose3d`][froglib.utils.arrayToPose3d] | Function | Constructs a `Pose3d` from a 6-element list `[x, y, z, roll, pitch, yaw]`. |
| [`Buffer`][froglib.utils.Buffer] | Class | A rolling `deque` buffer that discards `None` values and computes arithmetic means once a threshold is met. |
| [`PoseBuffer`][froglib.utils.PoseBuffer] | Class | Specialized `Buffer` for `Pose2d` instances, computing running mean poses and coordinate standard deviations. |
| [`GearStage`][froglib.utils.GearStage] | Class | Represents a single gear reduction stage with input and output tooth counts. |
| [`DriveTrain`][froglib.utils.DriveTrain] | Class | Models gear stages and wheel diameter to convert between motor rotations/RPS and linear distance/speed. |

## Usage Examples

### Robot-to-Target Geometry (`RobotRelativeTarget`)

```python
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d
from froglib.utils import RobotRelativeTarget

# Current robot position on field
robot_pose = Pose2d(2.0, 4.5, Rotation2d.fromDegrees(0))

# Target position (e.g. Speaker AprilTag at height of 1.45m)
target_pose = Pose3d(0.0, 5.5, 1.45, Rotation3d())

# Calculate geometric relationship
target_calc = RobotRelativeTarget(robotPose=robot_pose, targetPose=target_pose)

print(f"2D Distance: {target_calc.distance:.2f} m")
print(f"3D Straight-line Range: {target_calc.range:.2f} m")
print(f"Drive Heading: {target_calc.driveHeading.degrees():.1f}°")
print(f"Elevation Angle: {target_calc.elevation.degrees():.1f}°")
```

### Statistical Pose Smoothing (`PoseBuffer`)

```python
from wpimath.geometry import Pose2d, Rotation2d
from froglib.utils import PoseBuffer

# Buffer holding up to 25 poses, requiring at least 5 poses for valid statistics
pose_buffer = PoseBuffer(size=25, validLength=5)

# Append incoming vision poses
for x in [1.98, 2.02, 2.01, 2.00, 1.99]:
    pose_buffer.append(Pose2d(x, 3.5, Rotation2d.fromDegrees(45.0)))

# Compute statistical average and standard deviation
smoothed_pose = pose_buffer.pose_average()
x_noise = pose_buffer.x_stddev()
print(f"Smoothed Pose: X={smoothed_pose.X():.3f}m, Y={smoothed_pose.Y():.3f}m, StdDev={x_noise:.4f}")
```

### Drivetrain Conversions (`DriveTrain` and `GearStage`)

```python
from froglib.utils import DriveTrain, GearStage

# SDS MK4i L3 gear stages: (14, 50), (28, 16), (15, 45) with 4-inch (0.1016 m) wheel
drive_mech = DriveTrain(
    gear_stages=[
        GearStage(14, 50),
        GearStage(16, 28),
        GearStage(15, 45),
    ],
    wheel_diameter=0.1016
)

# Convert desired chassis velocity (4.0 m/s) to motor RPS
motor_rps = drive_mech.speed_to_input_rps(speed=4.0)
print(f"Required motor speed for 4.0 m/s: {motor_rps:.2f} RPS")

# Convert motor rotations to distance traveled
meters_traveled = drive_mech.rotations_to_distance(rotations=50.0)
print(f"Distance after 50 motor rotations: {meters_traveled:.2f} m")
```

## Reference

::: froglib.utils
