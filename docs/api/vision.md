# Vision API

## Overview

The `froglib.vision` module integrates computer vision into FRC robot control systems using [PhotonVision](https://photonvision.org/) and [RobotPy](https://robotpy.readthedocs.io/). It provides two primary vision workflows:

1. **Global Pose Estimation (`FROGPoseEstimator`)**: Fuses AprilTag fiducial detections with known 3D field layouts (`AprilTagFieldLayout`) to produce high-accuracy robot poses. It supports both multi-tag coprocessor estimation and lowest-ambiguity single-tag estimation, while publishing real-time telemetry (pose, standard deviations, target IDs) to NetworkTables for dashboard visualization and odometry integration.
2. **Object & Game-Piece Detection (`FROGDetector`)**: A lightweight vision pipeline that clusters detected game pieces (such as notes or balls) into spatial groups using vectorized NumPy operations without requiring heavier machine learning dependencies like scikit-learn. It identifies target density centroids and exposes WPILib Command `Trigger` instances for automated intake and targeting routines.

## Key Components

| Component | Type | Description |
|-----------|------|-------------|
| [`FROGCameraConfig`][froglib.vision.FROGCameraConfig] | Class | Configuration container holding the camera name and 3D mounting transform relative to the robot frame. |
| [`FROGPoseEstimator`][froglib.vision.FROGPoseEstimator] | Class | Wraps `PhotonCamera` and `PhotonPoseEstimator` to compute robot field poses and publish telemetry to NetworkTables. |
| [`DetectionResult`][froglib.vision.DetectionResult] | Dataclass | Holds centroid coordinates (`target_yaw`, `target_pitch`), concentration score, and detection count of a clustered group. |
| [`FROGDetector`][froglib.vision.FROGDetector] | Class | Extends `PhotonCamera` to filter object detections, perform angular clustering via NumPy, and provide command triggers. |

## Usage Examples

### AprilTag Pose Estimation

```python
from wpimath.geometry import Transform3d, Translation3d, Rotation3d
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from froglib.vision import FROGPoseEstimator

# 1. Load the official FRC field layout
field_layout = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

# 2. Define camera position relative to robot center (e.g. 0.25m forward, 0.5m up, pitched up 15 deg)
robot_to_camera = Transform3d(
    Translation3d(0.25, 0.0, 0.5),
    Rotation3d.fromDegrees(0.0, -15.0, 0.0)
)

# 3. Instantiate the pose estimator
pose_estimator = FROGPoseEstimator(
    fieldTags=field_layout,
    camera_name="front_camera",
    robotToCamera=robot_to_camera
)

# 4. Periodically poll for estimated pose (e.g., inside a subsystem's periodic method)
estimated_pose = pose_estimator.get_estimate()
if estimated_pose is not None:
    current_pose2d = estimated_pose.estimatedPose.toPose2d()
    timestamp = estimated_pose.timestampSeconds
    print(f"Pose estimate: X={current_pose2d.X():.2f}m, Y={current_pose2d.Y():.2f}m, Time={timestamp:.2f}s")
```

### Game-Piece Detection & Clustering

```python
from wpimath.geometry import Transform3d
from commands2 import Command
from froglib.vision import FROGCameraConfig, FROGDetector

# 1. Configure and instantiate the detector
config = FROGCameraConfig(name="note_camera", robotToCamera=Transform3d())
detector = FROGDetector(camera_config=config)

# 2. Update targets during periodic loop
def periodic():
    detector.get_targets()
    detector.process_targets()
    result = detector.get_detection_results()
    if result and detector.has_valid_target():
        print(f"Centroid: yaw={result.target_yaw:.1f}°, pitch={result.target_pitch:.1f}°, items={result.num_detections}")

# 3. Create a trigger that executes an intake command when game pieces are close
intake_trigger = detector.get_trigger_targets_close(yaw_threshold_deg=15.0, pitch_threshold_deg=-12.0)
# intake_trigger.onTrue(RunIntakeCommand())
```

## Reference

::: froglib.vision
