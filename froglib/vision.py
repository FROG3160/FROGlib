from dataclasses import dataclass
import numpy as np
from typing import List, Tuple, Optional, Dict
from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose
from photonlibpy.targeting import PhotonTrackedTarget
from wpimath.geometry import Transform3d
from wpimath.geometry import Pose3d, Pose2d
from wpiutil import Sendable, SendableBuilder
from FROGlib.utils import PoseBuffer
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
from commands2.button import Trigger


class FROGCameraConfig:
    def __init__(self, name: str, robotToCamera: Transform3d = Transform3d()):
        self.name = name
        self.robotToCamera = robotToCamera


class FROGPoseEstimator:
    def __init__(
        self,
        fieldTags: AprilTagFieldLayout,
        camera_name: str,
        robotToCamera: Transform3d = Transform3d(),
    ):
        self.camera = PhotonCamera(camera_name)
        self.fieldTags = fieldTags
        self.estimator = PhotonPoseEstimator(
            fieldTags,
            robotToCamera,
        )

        nt_table = f"FROGSubsystems/Vision/{camera_name}"
        self.pose_buffer = PoseBuffer(25)
        self._latest_pose_pub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/estimated_pose", Pose2d)
            .publish()
        )

        self._stdev_x_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_x")
            .publish()
        )
        self._stdev_y_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_y")
            .publish()
        )
        self._stdev_rotation_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/stdev_rotation")
            .publish()
        )
        self._has_targets_pub = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/has_targets")
            .publish()
        )
        self._tag_id_pub = (
            NetworkTableInstance.getDefault()
            .getIntegerTopic(f"{nt_table}/target_id")
            .publish()
        )
        self._target_ambiguity_rotation_pub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/target_ambiguity")
            .publish()
        )

    def get_estimate(self) -> EstimatedRobotPose | None:
        estPose = None
        for result in self.camera.getAllUnreadResults():
            estPose = self.estimator.estimateCoprocMultiTagPose(result)
            if estPose is None:
                estPose = self.estimator.estimateLowestAmbiguityPose(result)

        # if we have an estimated Pose, publish it, otherwise publish a bad Pose2d
        if estPose:
            self._latest_pose_pub.set(estPose.estimatedPose.toPose2d())
        else:
            self._latest_pose_pub.set(Pose2d(-1, -1, 0))

        return estPose

        # estimated_pose = self.update()
        # if estimated_pose:
        #     targets = estimated_pose.targetsUsed
        #     SmartDashboard.putNumber(
        #         f"{self._camera.getName()} Targets Found", len(targets)
        #     )
        #     target1 = targets[0]
        #     SmartDashboard.putNumber(
        #         f"{self._camera.getName()} First Target Ambiguity",
        #         target1.getPoseAmbiguity(),
        #     )

    def get_distance_to_tag(self, pose: Pose3d, tag_num: int) -> float | None:
        if tagpose := self.getTagPose(tag_num):
            return (
                pose.toPose2d().translation().distance(tagpose.toPose2d().translation())
            )
        else:
            return None

    def getTagPose(self, tag_id: int):
        return self.fieldTags.getTagPose(tag_id)

    # def periodic(self):

    #     self.latestVisionPose = self.update()
    #     result = self._camera.getLatestResult()
    #     if result.hasTargets():
    #         target = result.getBestTarget()
    #         ambiguity = target.getPoseAmbiguity()
    #         tag_id = target.getFiducialId()
    #         self._has_targets_pub.set(result.hasTargets())
    #         self._tag_id_pub.set(tag_id)
    #         self._target_ambiguity_rotation_pub.set(ambiguity)

    #     if self.latestVisionPose:
    #         self.pose_buffer.append(self.latestVisionPose.estimatedPose.toPose2d())
    #         # target_details = []
    #         # for photon_target in self.latestVisionPose.targetsUsed():
    #         #     target_details.append(
    #         #         {
    #         #             "tag": photon_target.fiducialId,
    #         #             "ambiguity": photon_target.poseAmbiguity,
    #         #             "transform": photon_target.bestCameraToTarget,
    #         #             "distance": self.get_distance_to_tag(
    #         #                 self.latestVisionPose.estimatedPose.toPose2d(),
    #         #                 photon_target.fiducialId,
    #         #             ),
    #         #         }
    #         #     )

    #         self._latest_pose_pub.set(self.latestVisionPose.estimatedPose.toPose2d())
    #         self._stdev_x_pub.set(self.pose_buffer.x_stddev())
    #         self._stdev_y_pub.set(self.pose_buffer.y_stddev())
    #         self._stdev_rotation_pub.set(self.pose_buffer.rotation_stddev())
    #         return self.latestVisionPose


@dataclass
class DetectionResult:
    """Result of detection processing"""

    target_yaw: float
    target_pitch: float
    # density_score: float  #sum of areas in cluster, weighted if enabled
    concentration_score: float  # sum of areas in cluster / concentration score
    num_detections: int  # number of targets in cluster?
    avg_confidence: float = 0.0


class FROGDetector(PhotonCamera):
    """
    Lightweight version of FROGDetection with no sklearn dependency.
    Uses only numpy for all clustering operations.
    """

    def __init__(
        self,
        camera_config: FROGCameraConfig,
    ):
        """
        Initialize the lightweight detection camera.

        Args:
            camera_name: Name of the PhotonVision camera
            use_weighted_centroid: Weight targets by area (True) or simple average (False)
            cluster_radius: Maximum distance (degrees) for targets to be in same cluster
        """
        super().__init__(camera_config.name)
        self._min_confidence = 0.8
        self._cluster_radius_deg = 12.0
        self._min_neighbors = 2
        self._use_area_weighting = True
        self.targets: List[PhotonTrackedTarget] = []
        self.detection_target: Optional[DetectionResult] = None
        # self.alt_detection_target: Optional[DetectionResult] = None
        self._yaws: np.ndarray = np.array([])
        self._pitches: np.ndarray = np.array([])
        self._areas: np.ndarray = np.array([])

    def get_targets(self):
        if results := self.getAllUnreadResults():
            result = results[-1]

            if not result.hasTargets():
                self.targets = []

            else:
                self.targets = result.getTargets()
        else:
            self.targets = []

    def process_targets(self):
        if self.targets is None:
            self._yaws = self._pitches = self._areas = np.array([])
            return
        valid = [t for t in self.targets if t.objDetectConf >= self._min_confidence]

        # Extract to numpy for other clustering and other analysis methods
        if valid:
            self._yaws = np.fromiter((t.yaw for t in valid), dtype=float)
            self._pitches = np.fromiter((t.pitch for t in valid), dtype=float)
            self._areas = np.fromiter((t.area for t in valid), dtype=float)
        else:
            self._yaws = self._pitches = self._areas = np.array([])

    def get_detection_results(
        self,
    ) -> Optional[DetectionResult] | None:

        if self.targets:
            self.detection_target = self.get_best_cluster(
                cluster_radius_deg=self._cluster_radius_deg,
                min_neighbors=self._min_neighbors,
                use_area_weighting=self._use_area_weighting,
            )
        else:
            return None
        return self.detection_target

    def get_best_cluster(
        self,
        cluster_radius_deg: float = 12.0,
        min_neighbors: int = 2,
        use_area_weighting: bool = True,
    ) -> Optional[DetectionResult]:

        # Calculate Distance Matrix (Broadcasting)
        dist_sq = (self._yaws[:, np.newaxis] - self._yaws[np.newaxis, :]) ** 2 + (
            self._pitches[:, np.newaxis] - self._pitches[np.newaxis, :]
        ) ** 2
        adj = dist_sq <= (cluster_radius_deg**2)

        # Score neighborhoods based on density
        if use_area_weighting:
            scores = adj @ self._areas
        else:
            scores = adj.sum(axis=1)

        best_idx = np.argmax(scores)

        # Validate against minimum neighbors
        num_neighbors = int(adj[best_idx].sum())
        if num_neighbors < min_neighbors:
            return None

        # Extract the winning cluster and calculate centroid
        cluster_mask = adj[best_idx]

        if use_area_weighting:
            weights = self._areas[cluster_mask]
            target_yaw = np.sum(self._yaws[cluster_mask] * weights) / weights.sum()
            target_pitch = np.sum(self._pitches[cluster_mask] * weights) / weights.sum()
        else:
            target_yaw = np.mean(self._yaws[cluster_mask])
            target_pitch = np.mean(self._pitches[cluster_mask])

        return DetectionResult(
            target_yaw=float(target_yaw),
            target_pitch=float(target_pitch),
            concentration_score=float(scores[best_idx]),
            num_detections=num_neighbors,
        )

    def has_valid_target(self) -> bool:
        """Check if there is a valid target concentration."""
        return self.detection_target is not None

    def get_concentration_info(self) -> str:
        """Get human-readable information about the current fuel concentration."""
        result = self.detection_target

        if result is None:
            return "No fuel detected"

        return (
            f"Detected {result.num_detections} fuel objects\n"
            f"Target: Yaw={result.target_yaw:.2f}°, Pitch={result.target_pitch:.2f}°\n"
            f"Concentration score: {result.concentration_score:.3f}"
        )

    def has_targets_close(
        self,
        yaw_threshold_deg: float = 20.0,
        pitch_threshold_deg: float = -10.0,
        min_area: float = 0.0,
    ) -> bool:
        if len(self._yaws) == 0:
            return False
        matches = (
            (np.abs(self._yaws) <= yaw_threshold_deg)
            & (self._pitches <= pitch_threshold_deg)
            & (self._areas >= min_area)
        )
        return bool(np.any(matches))

    def get_trigger_targets_close(
        self,
        yaw_threshold_deg: float = 20.0,
        pitch_threshold_deg: float = -10.0,
    ) -> Trigger:
        return Trigger(
            lambda: self.has_targets_close(yaw_threshold_deg, pitch_threshold_deg)
        )
