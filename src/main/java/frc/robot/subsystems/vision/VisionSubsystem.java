package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.statemachines.DriveState;
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

  public VisionSubsystem() {}

  @Override
  public void periodic() {
    if (driveState.hasDriveStats()) {
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
    }
  }

  // code from PoseEstimator class
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
      evaluateEstimation(
          estimatedPose,
          result.getTimestampSeconds(),
          multiTargetPNPResult.fiducialIDsUsed,
          cameraName);
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
          cameraName);
    }
  }

  // evaluates the estimations before export
  private void evaluateEstimation(
      Pose3d pose, double captureTime, List<Short> targetsUsed, String cameraName) {

    // field constraints
    if (pose.getX() < 0 || pose.getY() < 0 || pose.getX() > 16.7 || pose.getY() > 8.2) return;

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    if (!RobotModeTriggers.disabled().getAsBoolean()
        && !resettingPose
        && estimateDistance > VisionPreferences.jumpLimit.getValue()) {
      logBadResult(cameraName, "JUMPING");
      return;
    }

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
    if (resettingPose) {
      xyStdDev = 0;
      thetaStdDev = 0;
    }

    driveState.addVisionEstimate(
        new VisionMeasurement(
            pose.toPose2d(), captureTime, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)),
        cameraName);

    incrementAccepted(cameraName);
  }

  // evaluates the estimations before export
  private void evaluateEstimation(
      Pose3d pose, double captureTime, int targetUsed, String cameraName) {

    // field constraints
    if (pose.getX() < 0 || pose.getY() < 0 || pose.getX() > 16.7 || pose.getY() > 8.2) return;

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    if (!RobotModeTriggers.disabled().getAsBoolean()
        && !resettingPose
        && estimateDistance > VisionPreferences.jumpLimit.getValue()) return;

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

    if (resettingPose) {
      xyStdDev = 0;
      thetaStdDev = 0;
    }

    driveState.addVisionEstimate(
        new VisionMeasurement(
            pose.toPose2d(), captureTime, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)),
        cameraName);

    incrementAccepted(cameraName);
  }

  private void incrementAccepted(String cameraName) {
    if (cameraName.equals(VisionConstants.photonCameraName_Front)) frontCameraResultsAccepted++;
    if (cameraName.equals(VisionConstants.photonCameraName_Left)) leftCameraResultsAccepted++;
    if (cameraName.equals(VisionConstants.photonCameraName_Right)) rightCameraResultsAccepted++;
  }

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

  public Command resetPose() {
    return runOnce(() -> this.resettingPose = true);
  }

  public Command stopResetPose() {
    return runOnce(() -> this.resettingPose = false);
  }
}
