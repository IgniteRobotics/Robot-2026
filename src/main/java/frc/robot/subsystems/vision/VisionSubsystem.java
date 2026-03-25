package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.statemachines.DriveState;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class VisionSubsystem extends SubsystemBase {

  private DriveState driveState = DriveState.getInstance();
  private SwerveDriveState driveStats;

  @Logged(name = "Front Camera/Results Accepted", importance = Importance.CRITICAL)
  private long frontCameraResultsAccepted = 0;

  @Logged(name = "Front Camera/Results Rejected/Total", importance = Importance.CRITICAL)
  private long frontCameraResultsRejectedTotal = 0;

  @Logged(name = "Front Camera/Results Rejected/Ambiguity", importance = Importance.CRITICAL)
  private long frontCameraResultsRejectedDueToAmbiguity = 0;

  @Logged(name = "Front Camera/Results Rejected/Jumping", importance = Importance.CRITICAL)
  private long frontCameraResultsRejectedDueToJumping = 0;

  @Logged(name = "Left Camera/Results Accepted", importance = Importance.CRITICAL)
  private long leftCameraResultsAccepted = 0;

  @Logged(name = "Left Camera/Results Rejected/Total", importance = Importance.CRITICAL)
  private long leftCameraResultsRejectedTotal = 0;

  @Logged(name = "Left Camera/Results Rejected/Ambiguity", importance = Importance.CRITICAL)
  private long leftCameraResultsRejectedDueToAmbiguity = 0;

  @Logged(name = "Left Camera/Results Rejected/Jumping", importance = Importance.CRITICAL)
  private long leftCameraResultsRejectedDueToJumping = 0;

  @Logged(name = "Right Camera/Results Accepted", importance = Importance.CRITICAL)
  private long rightCameraResultsAccepted = 0;

  @Logged(name = "Right Camera/Results Rejected/Total", importance = Importance.CRITICAL)
  private long rightCameraResultsRejectedTotal = 0;

  @Logged(name = "Right Camera/Results Rejected/Ambiguity", importance = Importance.CRITICAL)
  private long rightCameraResultsRejectedDueToAmbiguity = 0;

  @Logged(name = "Right Camera/Results Rejected/Jumping", importance = Importance.CRITICAL)
  private long rightCameraResultsRejectedDueToJumping = 0;

  @Logged(name = "Resetting", importance = Importance.CRITICAL)
  private boolean resettingPose = false;

  // Clustering telemetry
  @Logged(name = "Clustering/Total Clusters Formed", importance = Importance.CRITICAL)
  private long totalClustersFormed = 0;

  @Logged(name = "Clustering/Single Camera Clusters", importance = Importance.CRITICAL)
  private long singleCameraClusters = 0;

  @Logged(name = "Clustering/Multi Camera Clusters", importance = Importance.CRITICAL)
  private long multiCameraClusters = 0;

  @Logged(name = "Clustering/Max Cluster Size", importance = Importance.CRITICAL)
  private int maxClusterSize = 0;

  @Logged(name = "Clustering/Outliers Rejected", importance = Importance.CRITICAL)
  private long outliersRejected = 0;

  @Logged(name = "Clustering/Drift Detection Active", importance = Importance.CRITICAL)
  private boolean driftDetectionActive = false;

  @Logged(name = "Clustering/Drift Consecutive Cycles", importance = Importance.CRITICAL)
  private int driftConsecutiveCycles = 0;

  @Logged(name = "Clustering/Estimates Processed This Cycle", importance = Importance.CRITICAL)
  private int estimatesProcessedThisCycle = 0;

  // Frame correlation
  @Logged(name = "Frame Info/Cycle Counter", importance = Importance.CRITICAL)
  private long frameCycleCounter = 0;

  @Logged(name = "Frame Info/Timestamp", importance = Importance.CRITICAL)
  private double frameTimestamp = 0;

  @Logged(name = "Frame Info/Active Cameras", importance = Importance.CRITICAL)
  private int activeCamerasThisFrame = 0;

  // Front Camera logging
  @Logged(name = "Front Camera/Pose", importance = Importance.CRITICAL)
  private Pose2d frontCameraPose = new Pose2d();

  @Logged(name = "Front Camera/Timestamp", importance = Importance.CRITICAL)
  private double frontCameraTimestamp = 0;

  @Logged(name = "Front Camera/Tag Count", importance = Importance.CRITICAL)
  private int frontCameraTagCount = 0;

  @Logged(name = "Front Camera/Ambiguity", importance = Importance.CRITICAL)
  private double frontCameraAmbiguity = 0;

  @Logged(name = "Front Camera/XY StdDev", importance = Importance.CRITICAL)
  private double frontCameraXyStdDev = 0;

  @Logged(name = "Front Camera/Theta StdDev", importance = Importance.CRITICAL)
  private double frontCameraThetaStdDev = 0;

  @Logged(name = "Front Camera/Average Distance", importance = Importance.CRITICAL)
  private double frontCameraAvgDistance = 0;

  @Logged(name = "Front Camera/Valid This Frame", importance = Importance.CRITICAL)
  private boolean frontCameraValidThisFrame = false;

  @Logged(name = "Front Camera/Results This Cycle", importance = Importance.CRITICAL)
  private int frontCameraResultsThisCycle = 0;

  // Left Camera logging
  @Logged(name = "Left Camera/Pose", importance = Importance.CRITICAL)
  private Pose2d leftCameraPose = new Pose2d();

  @Logged(name = "Left Camera/Timestamp", importance = Importance.CRITICAL)
  private double leftCameraTimestamp = 0;

  @Logged(name = "Left Camera/Tag Count", importance = Importance.CRITICAL)
  private int leftCameraTagCount = 0;

  @Logged(name = "Left Camera/Ambiguity", importance = Importance.CRITICAL)
  private double leftCameraAmbiguity = 0;

  @Logged(name = "Left Camera/XY StdDev", importance = Importance.CRITICAL)
  private double leftCameraXyStdDev = 0;

  @Logged(name = "Left Camera/Theta StdDev", importance = Importance.CRITICAL)
  private double leftCameraThetaStdDev = 0;

  @Logged(name = "Left Camera/Average Distance", importance = Importance.CRITICAL)
  private double leftCameraAvgDistance = 0;

  @Logged(name = "Left Camera/Valid This Frame", importance = Importance.CRITICAL)
  private boolean leftCameraValidThisFrame = false;

  @Logged(name = "Left Camera/Results This Cycle", importance = Importance.CRITICAL)
  private int leftCameraResultsThisCycle = 0;

  // Right Camera logging
  @Logged(name = "Right Camera/Pose", importance = Importance.CRITICAL)
  private Pose2d rightCameraPose = new Pose2d();

  @Logged(name = "Right Camera/Timestamp", importance = Importance.CRITICAL)
  private double rightCameraTimestamp = 0;

  @Logged(name = "Right Camera/Tag Count", importance = Importance.CRITICAL)
  private int rightCameraTagCount = 0;

  @Logged(name = "Right Camera/Ambiguity", importance = Importance.CRITICAL)
  private double rightCameraAmbiguity = 0;

  @Logged(name = "Right Camera/XY StdDev", importance = Importance.CRITICAL)
  private double rightCameraXyStdDev = 0;

  @Logged(name = "Right Camera/Theta StdDev", importance = Importance.CRITICAL)
  private double rightCameraThetaStdDev = 0;

  @Logged(name = "Right Camera/Average Distance", importance = Importance.CRITICAL)
  private double rightCameraAvgDistance = 0;

  @Logged(name = "Right Camera/Valid This Frame", importance = Importance.CRITICAL)
  private boolean rightCameraValidThisFrame = false;

  @Logged(name = "Right Camera/Results This Cycle", importance = Importance.CRITICAL)
  private int rightCameraResultsThisCycle = 0;

  // Fused result logging
  @Logged(name = "Fused/Pose", importance = Importance.CRITICAL)
  private Pose2d fusedPose = new Pose2d();

  @Logged(name = "Fused/Timestamp", importance = Importance.CRITICAL)
  private double fusedTimestamp = 0;

  @Logged(name = "Fused/Cluster Size", importance = Importance.CRITICAL)
  private int fusedClusterSize = 0;

  @Logged(name = "Fused/XY StdDev", importance = Importance.CRITICAL)
  private double fusedXyStdDev = 0;

  @Logged(name = "Fused/Theta StdDev", importance = Importance.CRITICAL)
  private double fusedThetaStdDev = 0;

  @Logged(name = "Fused/Was Fused This Frame", importance = Importance.CRITICAL)
  private boolean wasFusedThisFrame = false;

  @Logged(name = "Fused/Drift Detected", importance = Importance.CRITICAL)
  private boolean fusedDriftDetected = false;

  // Pending estimates for clustering
  private List<PendingEstimate> pendingEstimates = new ArrayList<>();

  // Temporary storage for best estimates this cycle (for logging)
  private PendingEstimate bestFrontEstimate = null;
  private PendingEstimate bestLeftEstimate = null;
  private PendingEstimate bestRightEstimate = null;

  // Temporary storage for best fused cluster this cycle (for logging)
  private VisionCluster bestFusedCluster = null;

  // Drift detection state
  private DriftDetectionState driftState = new DriftDetectionState();

  public class VisionMeasurement {
    private Pose2d estimatedPose;
    private double timestamp;
    private Vector<N3> trustValues;

    public VisionMeasurement(Pose2d pose, double time, Vector<N3> trust) {
      estimatedPose = pose;
      timestamp = time;
      trustValues = trust;
    }

    public Pose2d getEstimatedPose() {
      return estimatedPose;
    }

    public double getTimestamp() {
      return timestamp;
    }

    public Vector<N3> getTrust() {
      return trustValues;
    }
  }

  /** Stores a pending vision estimate with all metadata needed for clustering and fusion. */
  private class PendingEstimate {
    Pose2d pose;
    double timestamp;
    String cameraName;
    int tagCount;
    double xyStdDev;
    double thetaStdDev;
    double averageTagDistance;
    double ambiguity;

    PendingEstimate(
        Pose2d pose,
        double timestamp,
        String cameraName,
        int tagCount,
        double xyStdDev,
        double thetaStdDev,
        double averageTagDistance,
        double ambiguity) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.cameraName = cameraName;
      this.tagCount = tagCount;
      this.xyStdDev = xyStdDev;
      this.thetaStdDev = thetaStdDev;
      this.averageTagDistance = averageTagDistance;
      this.ambiguity = ambiguity;
    }
  }

  /** Represents a cluster of related vision estimates and their fused result. */
  private class VisionCluster {
    List<PendingEstimate> estimates = new ArrayList<>();
    Pose2d fusedPose;
    double fusedTimestamp;
    double fusedXyStdDev;
    double fusedThetaStdDev;
    boolean usesFront, usesLeft, usesRight;

    VisionCluster(PendingEstimate initialEstimate) {
      estimates.add(initialEstimate);
      if (initialEstimate.cameraName.equals(VisionConstants.photonCameraName_Front))
        usesFront = true;
      else usesFront = false;
      if (initialEstimate.cameraName.equals(VisionConstants.photonCameraName_Left)) usesLeft = true;
      else usesLeft = false;
      if (initialEstimate.cameraName.equals(VisionConstants.photonCameraName_Right))
        usesRight = true;
      else usesRight = false;
    }

    void addEstimate(PendingEstimate estimate) {
      estimates.add(estimate);
    }

    int size() {
      return estimates.size();
    }

    int camerasUsed() {
      int count = 0;
      if (usesFront) count++;
      if (usesLeft) count++;
      if (usesRight) count++;
      return count;
    }
  }

  /** Tracks drift detection state machine for automatic odometry drift detection. */
  private class DriftDetectionState {
    int consecutiveAgreementCycles = 0;
    Translation2d lastAgreementDirection = null;
    boolean driftDetected = false;

    void reset() {
      consecutiveAgreementCycles = 0;
      lastAgreementDirection = null;
      driftDetected = false;
    }
  }

  public VisionSubsystem() {}

  /**
   * Periodic method that processes vision measurements from all cameras.
   *
   * <p>Algorithm:
   *
   * <ol>
   *   <li>Captures current drive stats as a snapshot for consistent processing
   *   <li>Retrieves all unread results from each camera in FIFO order (oldest first)
   *   <li>Processes each result through evaluation pipeline to generate pose estimates
   *   <li>Tracks best estimate per camera (lowest ambiguity) and counts results per cycle
   *   <li>Performs clustering on pending estimates to detect multi-camera consensus
   *   <li>Submits fused or individual estimates to pose estimator
   *   <li>Finalizes logging with best per-camera and fused estimates
   * </ol>
   *
   * <p>This runs every robot loop cycle (~20ms) and processes multiple frames per camera if
   * available, ensuring no vision data is lost. When multiple results are processed per camera, the
   * best quality estimate (lowest ambiguity) is logged while all estimates contribute to
   * clustering.
   */
  @Override
  public void periodic() {
    if (driveState.hasDriveStats()) {
      // Clear pending estimates from previous cycle
      pendingEstimates.clear();

      // Increment frame counter and update timestamp
      frameCycleCounter++;
      frameTimestamp = Timer.getFPGATimestamp();

      // Reset per-frame flags
      frontCameraValidThisFrame = false;
      leftCameraValidThisFrame = false;
      rightCameraValidThisFrame = false;
      wasFusedThisFrame = false;
      activeCamerasThisFrame = 0;

      // Reset result counters
      frontCameraResultsThisCycle = 0;
      leftCameraResultsThisCycle = 0;
      rightCameraResultsThisCycle = 0;

      // Reset best estimate tracking
      bestFrontEstimate = null;
      bestLeftEstimate = null;
      bestRightEstimate = null;
      bestFusedCluster = null;

      // this makes sure that the different parts of the periodic use different stats
      driveStats = driveState.getCurrentDriveStats();

      // Front Camera
      // This is FIFO, so the oldest is given first and the newest last'
      var frontCameraResults = VisionConstants.photonCamera_Front.getAllUnreadResults();
      for (var result : frontCameraResults)
        evaluateResult(result, VisionConstants.photonCameraName_Front);

      // Left Camera
      var leftCameraResults = VisionConstants.photonCamera_Left.getAllUnreadResults();
      for (var result : leftCameraResults)
        evaluateResult(result, VisionConstants.photonCameraName_Left);

      // Right Camera
      var rightCameraResults = VisionConstants.photonCamera_Right.getAllUnreadResults();
      for (var result : rightCameraResults)
        evaluateResult(result, VisionConstants.photonCameraName_Right);

      // Perform clustering and submit estimates
      performClustering();

      // Finalize per-camera and fused logging with best estimates
      finalizePerCameraLogging();
      finalizeFusedLogging();
    }
  }

  /**
   * Evaluates a PhotonVision pipeline result and generates robot pose estimates.
   *
   * <p>Algorithm uses a two-tier approach:
   *
   * <p>Tier 1 - Multi-tag PNP (Preferred):
   *
   * <ul>
   *   <li>Uses multiple AprilTags simultaneously for more accurate pose estimation
   *   <li>Checks ambiguity score is below threshold (rejects if pose is uncertain)
   *   <li>Transforms camera-space pose to field-space:
   *       <ol>
   *         <li>Start with camera's estimated pose relative to tags
   *         <li>Convert to field coordinate system origin
   *         <li>Apply inverse camera transform to get robot center pose
   *       </ol>
   * </ul>
   *
   * <p>Tier 2 - Single-tag fallback:
   *
   * <ul>
   *   <li>Used when multi-tag fails or only one tag is visible
   *   <li>Selects the fiducial target with lowest pose ambiguity
   *   <li>Rejects if best target still exceeds ambiguity threshold
   *   <li>Calculates robot pose by:
   *       <ol>
   *         <li>Getting known tag position from field layout
   *         <li>Applying inverse camera-to-target transform
   *         <li>Applying inverse camera-to-robot transform
   *       </ol>
   * </ul>
   *
   * @param result PhotonVision pipeline result containing detected targets
   * @param cameraName Name of camera for transform lookup and logging
   */
  private void evaluateResult(PhotonPipelineResult result, String cameraName) {

    if (!result.multitagResult.isEmpty()
        && result.multitagResult.get().estimatedPose.ambiguity
            < VisionConstants.MAXIMUM_ALLOWED_AMBIGUITY) {
      MultiTargetPNPResult multiTargetPNPResult = result.multitagResult.get();
      Pose3d estimatedPose =
          Pose3d.kZero
              .plus(multiTargetPNPResult.estimatedPose.best)
              .relativeTo(VisionConstants.FIELD_LAYOUT.getOrigin())
              .plus(VisionConstants.cameraTransformMap.get(cameraName).inverse());
      double ambiguity = multiTargetPNPResult.estimatedPose.ambiguity;
      evaluateEstimation(
          estimatedPose,
          result.getTimestampSeconds(),
          multiTargetPNPResult.fiducialIDsUsed,
          cameraName,
          ambiguity);
    } else if (result.hasTargets()) {
      PhotonTrackedTarget lowestAmbiguityTarget = null;
      double lowestAmbiguityScore = 10;

      for (PhotonTrackedTarget target : result.targets) {
        double targetPoseAmbiguity = target.getPoseAmbiguity();
        // Make sure the target is a Fiducial target.
        if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
          lowestAmbiguityScore = targetPoseAmbiguity;
          lowestAmbiguityTarget = target;
        }
      }

      // Although there are confirmed to be targets, none of them may be fiducial
      // targets.
      if (lowestAmbiguityScore > VisionConstants.MAXIMUM_ALLOWED_AMBIGUITY) {
        logBadResult(cameraName, "AMBIGUITY");
        return;
      }

      int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

      Pose3d targetPosition = VisionConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

      evaluateEstimation(
          targetPosition
              .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
              .transformBy(VisionConstants.cameraTransformMap.get(cameraName).inverse()),
          result.getTimestampSeconds(),
          targetFiducialId,
          cameraName,
          lowestAmbiguityScore);
    }
  }

  /**
   * Evaluates and filters multi-tag pose estimates before adding to pending list.
   *
   * <p>Filtering Algorithm:
   *
   * <ol>
   *   <li>Field boundary check - Rejects poses outside 16.7m x 8.2m field
   * </ol>
   *
   * <p>Uncertainty Calculation (Kalman Filter Standard Deviations):
   *
   * <ul>
   *   <li>XY Standard Deviation:
   *       <ul>
   *         <li>Base formula: coefficient × distance^1.2 / tag_count
   *         <li>Increases with distance (tags farther = less accurate)
   *         <li>Decreases with tag count (more tags = more confident)
   *         <li>Power of 1.2 models non-linear decrease in accuracy
   *       </ul>
   *   <li>Theta (rotation) Standard Deviation:
   *       <ul>
   *         <li>Same distance/tag_count scaling as XY
   *         <li>Additional penalty for robot rotation speed
   *         <li>Rotating robot = motion blur = less accurate heading
   *         <li>Formula: base × (1 + angular_velocity × omega_penalty)
   *       </ul>
   *   <li>During pose reset: Both set to 0.001 (near-zero uncertainty = full trust in vision)
   * </ul>
   *
   * <p>Note: Jump detection is now handled in clustering logic after multi-camera consensus.
   *
   * @param pose The estimated robot pose in 3D space
   * @param captureTime Timestamp when the camera captured the image
   * @param targetsUsed List of AprilTag IDs used in the multi-tag estimate
   * @param cameraName Name of camera for logging purposes
   * @param ambiguity PhotonVision ambiguity score for this estimate
   */
  private void evaluateEstimation(
      Pose3d pose,
      double captureTime,
      List<Short> targetsUsed,
      String cameraName,
      double ambiguity) {

    // field constraints
    if (pose.getX() < 0 || pose.getY() < 0 || pose.getX() > 16.7 || pose.getY() > 8.2) return;

    double averageRobotToTagDistance = 0;
    for (int target : targetsUsed)
      averageRobotToTagDistance +=
          pose.getTranslation()
                  .getDistance(
                      VisionConstants.FIELD_LAYOUT.getTagPose(target).get().getTranslation())
              / targetsUsed.size();

    double xyStdDev =
        VisionPreferences.xyStdDevCoef.getValue()
            * Math.pow(averageRobotToTagDistance, 1.2)
            / targetsUsed.size();

    double thetaStdDev =
        VisionPreferences.thetaStdDevCoef.getValue()
            * (1
                + driveStats.Speeds.omegaRadiansPerSecond
                    * VisionPreferences.omegaPenalty.getValue())
            * Math.pow(averageRobotToTagDistance, 1.2)
            / targetsUsed.size();

    // if we're resetting the pose, trust the vision measurements
    // Use very small epsilon instead of 0 to avoid divide-by-zero in fusion
    if (resettingPose) {
      xyStdDev = 0.001;
      thetaStdDev = 0.001;
    }

    // Add to pending estimates for clustering
    PendingEstimate estimate =
        new PendingEstimate(
            pose.toPose2d(),
            captureTime,
            cameraName,
            targetsUsed.size(),
            xyStdDev,
            thetaStdDev,
            averageRobotToTagDistance,
            ambiguity);
    pendingEstimates.add(estimate);

    // Update per-camera logging for AdvantageScope
    updateCameraLogging(estimate);

    incrementAccepted(cameraName);
  }

  /**
   * Evaluates and filters single-tag pose estimates before adding to pending list.
   *
   * <p>Filtering Algorithm:
   *
   * <ol>
   *   <li>Field boundary check - Rejects poses outside 16.7m x 8.2m field
   * </ol>
   *
   * <p>Uncertainty Calculation (Kalman Filter Standard Deviations):
   *
   * <ul>
   *   <li>XY Standard Deviation:
   *       <ul>
   *         <li>Formula: coefficient × distance^1.2
   *         <li>No tag count divisor (single tag = inherently less certain)
   *         <li>Increases non-linearly with distance
   *       </ul>
   *   <li>Theta (rotation) Standard Deviation:
   *       <ul>
   *         <li>Same distance scaling as XY
   *         <li>Additional penalty for robot rotation speed
   *         <li>Formula: base × (1 + angular_velocity × omega_penalty) × distance^1.2
   *       </ul>
   *   <li>During pose reset: Both set to 0.001 (near-zero uncertainty = full trust in vision)
   * </ul>
   *
   * <p>Note: Single-tag estimates are less reliable than multi-tag, so standard deviations will be
   * higher for the same distance (no division by tag count).
   *
   * <p>Note: Jump detection is now handled in clustering logic after multi-camera consensus.
   *
   * @param pose The estimated robot pose in 3D space
   * @param captureTime Timestamp when the camera captured the image
   * @param targetUsed AprilTag ID used for the single-tag estimate
   * @param cameraName Name of camera for logging purposes
   * @param ambiguity PhotonVision ambiguity score for this estimate
   */
  private void evaluateEstimation(
      Pose3d pose, double captureTime, int targetUsed, String cameraName, double ambiguity) {

    // field constraints
    if (pose.getX() < 0 || pose.getY() < 0 || pose.getX() > 16.7 || pose.getY() > 8.2) return;

    double robotToTagDistance =
        pose.getTranslation()
            .getDistance(
                VisionConstants.FIELD_LAYOUT.getTagPose(targetUsed).get().getTranslation());

    double xyStdDev = VisionPreferences.xyStdDevCoef.getValue() * Math.pow(robotToTagDistance, 1.2);

    double thetaStdDev =
        VisionPreferences.thetaStdDevCoef.getValue()
            * (1
                + driveStats.Speeds.omegaRadiansPerSecond
                    * VisionPreferences.omegaPenalty.getValue())
            * Math.pow(robotToTagDistance, 1.2);

    // Use very small epsilon instead of 0 to avoid divide-by-zero in fusion
    if (resettingPose) {
      xyStdDev = 0.001;
      thetaStdDev = 0.001;
    }

    // Add to pending estimates for clustering
    PendingEstimate estimate =
        new PendingEstimate(
            pose.toPose2d(),
            captureTime,
            cameraName,
            1,
            xyStdDev,
            thetaStdDev,
            robotToTagDistance,
            ambiguity);
    pendingEstimates.add(estimate);

    // Update per-camera logging for AdvantageScope
    updateCameraLogging(estimate);

    incrementAccepted(cameraName);
  }

  /**
   * Increments the accepted result counter for the specified camera. Used for telemetry and
   * monitoring vision system performance.
   *
   * @param cameraName Name of camera whose counter should be incremented
   */
  private void incrementAccepted(String cameraName) {
    if (cameraName.equals(VisionConstants.photonCameraName_Front)) frontCameraResultsAccepted++;
    if (cameraName.equals(VisionConstants.photonCameraName_Left)) leftCameraResultsAccepted++;
    if (cameraName.equals(VisionConstants.photonCameraName_Right)) rightCameraResultsAccepted++;
  }

  /**
   * Logs rejected vision results by camera and rejection reason. Tracks telemetry for monitoring
   * vision system health and tuning filters.
   *
   * @param cameraName Name of camera that produced the rejected result
   * @param type Rejection reason: "AMBIGUITY" (uncertain pose) or "JUMPING" (teleportation)
   */
  private void logBadResult(String cameraName, String type) {
    if (cameraName.equals(VisionConstants.photonCameraName_Front)) {
      frontCameraResultsRejectedTotal++;
      if (type.equals("AMBIGUITY")) frontCameraResultsRejectedDueToAmbiguity++;
      else if (type.equals("JUMPING")) frontCameraResultsRejectedDueToJumping++;
    } else if (cameraName.equals(VisionConstants.photonCameraName_Left)) {
      leftCameraResultsRejectedTotal++;
      if (type.equals("AMBIGUITY")) leftCameraResultsRejectedDueToAmbiguity++;
      else if (type.equals("JUMPING")) leftCameraResultsRejectedDueToJumping++;
    } else if (cameraName.equals(VisionConstants.photonCameraName_Right)) {
      rightCameraResultsRejectedTotal++;
      if (type.equals("AMBIGUITY")) rightCameraResultsRejectedDueToAmbiguity++;
      else if (type.equals("JUMPING")) rightCameraResultsRejectedDueToJumping++;
    }
  }

  /**
   * Checks if an estimate should be rejected due to jumping (teleportation).
   *
   * <p>Jump detection is disabled when:
   *
   * <ul>
   *   <li>Robot is disabled (no odometry baseline)
   *   <li>Pose reset is active (intentionally accepting large corrections)
   * </ul>
   *
   * @param estimatedPose The vision-estimated robot pose
   * @param cameraName Name of camera for logging
   * @return true if estimate should be rejected due to jumping
   */
  private boolean shouldRejectForJumping(Pose2d estimatedPose, String cameraName) {
    if (RobotModeTriggers.disabled().getAsBoolean() || resettingPose) {
      return false;
    }

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(estimatedPose.getTranslation());

    if (estimateDistance > VisionPreferences.jumpLimit.getValue()) {
      logBadResult(cameraName, "JUMPING");
      return true;
    }

    return false;
  }

  /**
   * Checks if two estimates are close enough to be clustered together.
   *
   * <p>Uses both distance and angle thresholds to determine clustering eligibility.
   *
   * @param e1 First estimate
   * @param e2 Second estimate
   * @return true if estimates should be clustered together
   */
  private boolean areEstimatesClose(PendingEstimate e1, PendingEstimate e2) {
    double distance = e1.pose.getTranslation().getDistance(e2.pose.getTranslation());
    double angleDiff = Math.abs(e1.pose.getRotation().minus(e2.pose.getRotation()).getRadians());

    return distance < VisionPreferences.clusterDistanceThreshold.getValue()
        && angleDiff < VisionPreferences.clusterAngleThreshold.getValue();
  }

  /**
   * Groups pending estimates into clusters using greedy distance-based algorithm.
   *
   * <p>Algorithm:
   *
   * <ol>
   *   <li>For each unclustered estimate, create a new cluster
   *   <li>Find all other unclustered estimates within threshold distance
   *   <li>Add neighbors to cluster and mark as clustered
   *   <li>Repeat until all estimates are clustered
   * </ol>
   *
   * <p>Complexity: O(n²) where n ≤ 3 cameras, so negligible overhead
   *
   * @param estimates List of pending estimates to cluster
   * @return List of clusters, each containing 1+ estimates
   */
  private List<VisionCluster> findClusters(List<PendingEstimate> estimates) {
    List<VisionCluster> clusters = new ArrayList<>();
    boolean[] clustered = new boolean[estimates.size()];

    for (int i = 0; i < estimates.size(); i++) {
      if (clustered[i]) continue;

      VisionCluster cluster = new VisionCluster(estimates.get(i));
      clustered[i] = true;

      // Find all neighbors within threshold
      for (int j = i + 1; j < estimates.size(); j++) {
        if (!clustered[j] && areEstimatesClose(estimates.get(i), estimates.get(j))) {
          cluster.addEstimate(estimates.get(j));
          clustered[j] = true;
          if (estimates.get(j).cameraName.equals(VisionConstants.photonCameraName_Front))
            cluster.usesFront = true;
          if (estimates.get(j).cameraName.equals(VisionConstants.photonCameraName_Left))
            cluster.usesLeft = true;
          if (estimates.get(j).cameraName.equals(VisionConstants.photonCameraName_Right))
            cluster.usesRight = true;
        }
      }

      clusters.add(cluster);
    }

    return clusters;
  }

  /**
   * Fuses multiple pose estimates in a cluster using inverse variance weighting.
   *
   * <p>Fusion Algorithm:
   *
   * <ol>
   *   <li>Calculate inverse variance weights: weight = 1/σ²
   *   <li>Compute weighted average of X, Y coordinates
   *   <li>Compute weighted average of rotation using sin/cos (handles wraparound)
   *   <li>Use oldest timestamp in cluster (conservative)
   *   <li>Apply LINEAR trust scaling: finalStdDev = avgStdDev / clusterSize
   * </ol>
   *
   * @param cluster Cluster to fuse
   */
  private void fusePosesInCluster(VisionCluster cluster) {
    if (cluster.size() == 1) {
      // No fusion needed for single estimate
      PendingEstimate e = cluster.estimates.get(0);
      cluster.fusedPose = e.pose;
      cluster.fusedTimestamp = e.timestamp;
      cluster.fusedXyStdDev = e.xyStdDev;
      cluster.fusedThetaStdDev = e.thetaStdDev;
      return;
    }

    // Calculate inverse variance weights
    double totalXyWeight = 0;
    double totalThetaWeight = 0;
    double weightedX = 0;
    double weightedY = 0;
    double weightedSin = 0;
    double weightedCos = 0;
    double oldestTimestamp = Double.MAX_VALUE;
    double sumXyStdDev = 0;
    double sumThetaStdDev = 0;

    for (PendingEstimate e : cluster.estimates) {
      double xyWeight = 1.0 / (e.xyStdDev * e.xyStdDev);
      double thetaWeight = 1.0 / (e.thetaStdDev * e.thetaStdDev);

      weightedX += e.pose.getX() * xyWeight;
      weightedY += e.pose.getY() * xyWeight;
      weightedSin += Math.sin(e.pose.getRotation().getRadians()) * thetaWeight;
      weightedCos += Math.cos(e.pose.getRotation().getRadians()) * thetaWeight;

      totalXyWeight += xyWeight;
      totalThetaWeight += thetaWeight;
      sumXyStdDev += e.xyStdDev;
      sumThetaStdDev += e.thetaStdDev;

      if (e.timestamp < oldestTimestamp) {
        oldestTimestamp = e.timestamp;
      }
    }

    // Compute fused pose
    double fusedX = weightedX / totalXyWeight;
    double fusedY = weightedY / totalXyWeight;
    double fusedTheta = Math.atan2(weightedSin / totalThetaWeight, weightedCos / totalThetaWeight);

    cluster.fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));
    cluster.fusedTimestamp = oldestTimestamp;

    // Apply LINEAR trust scaling
    double avgXyStdDev = sumXyStdDev / cluster.size();
    double avgThetaStdDev = sumThetaStdDev / cluster.size();

    cluster.fusedXyStdDev =
        (avgXyStdDev / cluster.size()) * VisionPreferences.trustScalingFactor.getValue();
    cluster.fusedThetaStdDev =
        (avgThetaStdDev / cluster.size()) * VisionPreferences.trustScalingFactor.getValue();
  }

  /**
   * Checks for odometry drift using multi-camera consensus.
   *
   * <p>Drift Detection Algorithm:
   *
   * <ol>
   *   <li>Calculate jump vector from odometry to cluster center
   *   <li>If distance < convergenceThreshold: reset drift state (converged)
   *   <li>If distance < minimumDistance: ignore (too small to analyze)
   *   <li>If direction consistent for N cycles: set driftDetected flag
   *   <li>If direction changes: reset counter
   * </ol>
   *
   * <p>Direction checking can be disabled by setting driftDirectionTolerance to 0, which will
   * trigger drift based purely on error magnitude exceeding minimumDistance for N cycles.
   *
   * @param cluster Vision cluster to check against odometry
   * @return true if drift detected (should bypass jump detection)
   */
  private boolean checkForDrift(VisionCluster cluster) {
    Translation2d jumpVector =
        cluster.fusedPose.getTranslation().minus(driveStats.Pose.getTranslation());
    double jumpDistance = jumpVector.getNorm();

    // Check for convergence (drift resolved)
    if (jumpDistance < VisionPreferences.driftConvergenceThreshold.getValue()) {
      driftState.reset();
      return false;
    }

    // Ignore small jumps
    if (jumpDistance < VisionPreferences.driftMinimumDistance.getValue()) {
      return driftState.driftDetected;
    }

    // Check direction consistency (if enabled)
    double directionTolerance = VisionPreferences.driftDirectionTolerance.getValue();
    boolean directionCheckEnabled = directionTolerance > 0;

    if (driftState.lastAgreementDirection == null) {
      // First detection
      driftState.lastAgreementDirection = jumpVector;
      driftState.consecutiveAgreementCycles = 1;
    } else {
      boolean directionConsistent = true;

      if (directionCheckEnabled) {
        // Check if direction is consistent
        double directionDiff = jumpVector.minus(driftState.lastAgreementDirection).getNorm();
        directionConsistent = directionDiff < directionTolerance;
      }
      // If direction check disabled (tolerance = 0), always consider consistent

      if (directionConsistent) {
        // Direction consistent (or check disabled), increment counter
        driftState.consecutiveAgreementCycles++;
        if (driftState.consecutiveAgreementCycles
            >= VisionPreferences.driftDetectionCycles.getValue()) {
          driftState.driftDetected = true;
        }
      } else {
        // Direction changed, reset
        driftState.lastAgreementDirection = jumpVector;
        driftState.consecutiveAgreementCycles = 1;
        driftState.driftDetected = false;
      }
    }

    return driftState.driftDetected;
  }

  /**
   * Performs clustering on pending estimates and submits fused results to pose estimator.
   *
   * <p>Algorithm handles three cases:
   *
   * <ol>
   *   <li>Single estimate (1 camera):
   *       <ul>
   *         <li>Apply normal jump detection
   *         <li>Submit with original uncertainty
   *       </ul>
   *   <li>Multiple estimates, single-camera clusters (outliers):
   *       <ul>
   *         <li>Apply normal jump detection
   *         <li>Reject if jumping
   *         <li>Submit individual estimates
   *       </ul>
   *   <li>Multiple estimates, multi-camera clusters (consensus):
   *       <ul>
   *         <li>Fuse poses via inverse variance weighting
   *         <li>Track best cluster (largest size, or lowest uncertainty if tied)
   *         <li>Check for drift detection
   *         <li>Apply jump detection (may be bypassed if drift detected)
   *         <li>Submit fused estimate with scaled uncertainty
   *       </ul>
   * </ol>
   *
   * <p>Note: When multiple multi-camera clusters are processed, all are submitted to the pose
   * estimator, but only the best cluster is logged (updated in finalizeFusedLogging()).
   */
  private void performClustering() {
    estimatesProcessedThisCycle = pendingEstimates.size();

    if (pendingEstimates.isEmpty()) {
      return;
    }

    // Handle single estimate case
    if (pendingEstimates.size() == 1) {
      PendingEstimate estimate = pendingEstimates.get(0);

      if (shouldRejectForJumping(estimate.pose, estimate.cameraName)) {
        return;
      }

      driveState.addVisionEstimate(
          new VisionMeasurement(
              estimate.pose,
              estimate.timestamp,
              VecBuilder.fill(estimate.xyStdDev, estimate.xyStdDev, estimate.thetaStdDev)),
          1);

      singleCameraClusters++;
      totalClustersFormed++;
      return;
    }

    // Multiple estimates - perform clustering
    List<VisionCluster> clusters = findClusters(pendingEstimates);
    totalClustersFormed += clusters.size();

    for (VisionCluster cluster : clusters) {
      fusePosesInCluster(cluster);

      // Update max cluster size
      if (cluster.size() > maxClusterSize) {
        maxClusterSize = cluster.size();
      }

      if (cluster.size() == 1) {
        // Single-camera cluster (outlier)
        PendingEstimate estimate = cluster.estimates.get(0);

        if (shouldRejectForJumping(estimate.pose, estimate.cameraName)) {
          outliersRejected++;
          continue;
        }

        driveState.addVisionEstimate(
            new VisionMeasurement(
                estimate.pose,
                estimate.timestamp,
                VecBuilder.fill(estimate.xyStdDev, estimate.xyStdDev, estimate.thetaStdDev)),
            1);

        singleCameraClusters++;
      } else {
        // Multi-camera cluster (consensus)
        boolean driftDetected = checkForDrift(cluster);

        // Update telemetry
        driftDetectionActive = driftState.driftDetected;
        driftConsecutiveCycles = driftState.consecutiveAgreementCycles;

        // Track best multi-camera cluster (largest, or lowest uncertainty if tied)
        if (bestFusedCluster == null
            || cluster.size() > bestFusedCluster.size()
            || (cluster.size() == bestFusedCluster.size()
                && cluster.fusedXyStdDev < bestFusedCluster.fusedXyStdDev)) {
          bestFusedCluster = cluster;
        }

        // Apply jump detection unless drift is detected
        if (!driftDetected && shouldRejectForJumping(cluster.fusedPose, "FUSED")) {
          continue;
        }

        // Submit fused estimate
        driveState.addVisionEstimate(
            new VisionMeasurement(
                cluster.fusedPose,
                cluster.fusedTimestamp,
                VecBuilder.fill(
                    cluster.fusedXyStdDev, cluster.fusedXyStdDev, cluster.fusedThetaStdDev)),
            cluster.camerasUsed());

        multiCameraClusters++;
      }
    }
  }

  /**
   * Tracks the best estimate for each camera (lowest ambiguity) for logging.
   *
   * <p>Instead of immediately updating logged fields, this method tracks the best estimate per
   * camera. The actual logged fields are updated at the end of periodic() via
   * finalizePerCameraLogging().
   *
   * <p>This prevents data loss when multiple results are processed per camera in a single cycle.
   *
   * @param estimate The pending estimate to track
   */
  private void updateCameraLogging(PendingEstimate estimate) {
    activeCamerasThisFrame++;

    if (estimate.cameraName.equals(VisionConstants.photonCameraName_Front)) {
      frontCameraResultsThisCycle++;
      // Keep estimate with lowest ambiguity
      if (bestFrontEstimate == null || estimate.ambiguity < bestFrontEstimate.ambiguity) {
        bestFrontEstimate = estimate;
      }
    } else if (estimate.cameraName.equals(VisionConstants.photonCameraName_Left)) {
      leftCameraResultsThisCycle++;
      if (bestLeftEstimate == null || estimate.ambiguity < bestLeftEstimate.ambiguity) {
        bestLeftEstimate = estimate;
      }
    } else if (estimate.cameraName.equals(VisionConstants.photonCameraName_Right)) {
      rightCameraResultsThisCycle++;
      if (bestRightEstimate == null || estimate.ambiguity < bestRightEstimate.ambiguity) {
        bestRightEstimate = estimate;
      }
    }
  }

  /**
   * Finalizes per-camera logging by updating logged fields with the best estimate from this cycle.
   *
   * <p>Called at end of periodic() after all camera results have been processed. Updates logged
   * fields with the estimate that had the lowest ambiguity (highest quality) for each camera.
   */
  private void finalizePerCameraLogging() {
    // Front camera
    if (bestFrontEstimate != null) {
      frontCameraPose = bestFrontEstimate.pose;
      frontCameraTimestamp = bestFrontEstimate.timestamp;
      frontCameraTagCount = bestFrontEstimate.tagCount;
      frontCameraAmbiguity = bestFrontEstimate.ambiguity;
      frontCameraXyStdDev = bestFrontEstimate.xyStdDev;
      frontCameraThetaStdDev = bestFrontEstimate.thetaStdDev;
      frontCameraAvgDistance = bestFrontEstimate.averageTagDistance;
      frontCameraValidThisFrame = true;
    }

    // Left camera
    if (bestLeftEstimate != null) {
      leftCameraPose = bestLeftEstimate.pose;
      leftCameraTimestamp = bestLeftEstimate.timestamp;
      leftCameraTagCount = bestLeftEstimate.tagCount;
      leftCameraAmbiguity = bestLeftEstimate.ambiguity;
      leftCameraXyStdDev = bestLeftEstimate.xyStdDev;
      leftCameraThetaStdDev = bestLeftEstimate.thetaStdDev;
      leftCameraAvgDistance = bestLeftEstimate.averageTagDistance;
      leftCameraValidThisFrame = true;
    }

    // Right camera
    if (bestRightEstimate != null) {
      rightCameraPose = bestRightEstimate.pose;
      rightCameraTimestamp = bestRightEstimate.timestamp;
      rightCameraTagCount = bestRightEstimate.tagCount;
      rightCameraAmbiguity = bestRightEstimate.ambiguity;
      rightCameraXyStdDev = bestRightEstimate.xyStdDev;
      rightCameraThetaStdDev = bestRightEstimate.thetaStdDev;
      rightCameraAvgDistance = bestRightEstimate.averageTagDistance;
      rightCameraValidThisFrame = true;
    }
  }

  /**
   * Finalizes fused logging by updating logged fields with the best cluster from this cycle.
   *
   * <p>Called at end of periodic() after clustering. Updates logged fields with the largest
   * multi-camera cluster (or lowest uncertainty if tied).
   */
  private void finalizeFusedLogging() {
    if (bestFusedCluster != null) {
      fusedPose = bestFusedCluster.fusedPose;
      fusedTimestamp = bestFusedCluster.fusedTimestamp;
      fusedClusterSize = bestFusedCluster.size();
      fusedXyStdDev = bestFusedCluster.fusedXyStdDev;
      fusedThetaStdDev = bestFusedCluster.fusedThetaStdDev;
      wasFusedThisFrame = true;

      // Check drift for best cluster
      fusedDriftDetected = checkForDrift(bestFusedCluster);
    }
  }

  /**
   * Command to enable pose reset mode.
   *
   * <p>When active, vision measurements are fully trusted (standard deviations = 0) and jump
   * detection is disabled. This allows the robot to accept large pose corrections, typically used
   * during autonomous initialization or manual reset.
   *
   * @return Command that sets resettingPose flag to true
   */
  public Command resetPose() {
    return runOnce(() -> this.resettingPose = true);
  }

  /**
   * Command to disable pose reset mode and return to normal vision processing.
   *
   * <p>Re-enables jump detection and distance-based uncertainty calculations. Should be called
   * after the pose has been successfully reset.
   *
   * @return Command that sets resettingPose flag to false
   */
  public Command stopResetPose() {
    return runOnce(() -> this.resettingPose = false);
  }
}
