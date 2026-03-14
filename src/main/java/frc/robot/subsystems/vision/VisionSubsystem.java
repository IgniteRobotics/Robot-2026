package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import java.util.List;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class VisionSubsystem extends SubsystemBase {

  private DriveState driveState = DriveState.getInstance();
  private SwerveDriveState driveStats;

  /*
  @Logged(name = "Highest Ambiguity Front Camera", importance = Importance.CRITICAL)
  private double highestAmbiguityFront;

  @Logged(name = "Highest Ambiguity Left Camera", importance = Importance.CRITICAL)
  private double highestAmbiguityLeft;

  @Logged(name = "Highest Ambiguity Right Camera", importance = Importance.CRITICAL)
  private double highestAmbiguityRight;
  */

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
      var frontCameraResults = CameraConstants.photonCamera_Front.getAllUnreadResults();
      for (var result : frontCameraResults)
        evaluateResult(result, CameraConstants.photonCameraName_Front);

      // Left Camera
      var leftCameraResults = CameraConstants.photonCamera_Left.getAllUnreadResults();
      for (var result : leftCameraResults)
        evaluateResult(result, CameraConstants.photonCameraName_Left);

      // Right Camera
      var rightCameraResults = CameraConstants.photonCamera_Right.getAllUnreadResults();
      for (var result : rightCameraResults)
        evaluateResult(result, CameraConstants.photonCameraName_Right);
    }
  }

  // code from PoseEstimator class
  private void evaluateResult(PhotonPipelineResult result, String cameraName) {

    if (!result.multitagResult.isEmpty()
        && result.multitagResult.get().estimatedPose.ambiguity
            < CameraConstants.MAXIMUM_ALLOWED_AMBIGUITY) {
      MultiTargetPNPResult multiTargetPNPResult = result.multitagResult.get();
      Pose3d estimatedPose =
          Pose3d.kZero
              .plus(multiTargetPNPResult.estimatedPose.best)
              .relativeTo(CameraConstants.FIELD_LAYOUT.getOrigin())
              .plus(CameraConstants.cameraTransformMap.get(cameraName).inverse());
      evaluateEstimation(estimatedPose, multiTargetPNPResult.fiducialIDsUsed, cameraName);
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
      if (lowestAmbiguityScore > CameraConstants.MAXIMUM_ALLOWED_AMBIGUITY) return;

      int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

      Pose3d targetPosition = CameraConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

      evaluateEstimation(
          targetPosition
              .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
              .transformBy(CameraConstants.cameraTransformMap.get(cameraName).inverse()),
          targetFiducialId,
          cameraName);
    }
  }

  // evaluates the estimations before export
  private void evaluateEstimation(Pose3d pose, List<Short> targetsUsed, String cameraName) {

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    if (!RobotModeTriggers.disabled().getAsBoolean()
        && estimateDistance > TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02) return;

    double averageRobotToTagDistance = 0;
    for (int target : targetsUsed)
      averageRobotToTagDistance +=
          pose.getTranslation()
                  .getDistance(
                      CameraConstants.FIELD_LAYOUT.getTagPose(target).get().getTranslation())
              / targetsUsed.size();

    double xyStdDev =
        CameraConstants.XY_STD_DEV_COEFFICIENT
            * Math.pow(averageRobotToTagDistance, 1.2)
            / Math.pow(targetsUsed.size(), 2);

    double thetaStdDev =
        CameraConstants.THETA_STD_DEV_COEFFICIENT
            * Math.pow(averageRobotToTagDistance, 1.2)
            / Math.pow(targetsUsed.size(), 2);

    driveState.addVisionEstimate(
        new VisionMeasurement(
            pose.toPose2d(),
            Utils.getCurrentTimeSeconds(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)),
        cameraName);
  }

  // evaluates the estimations before export
  private void evaluateEstimation(Pose3d pose, int targetUsed, String cameraName) {

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    if (!RobotModeTriggers.disabled().getAsBoolean()
        && estimateDistance > TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02) return;

    double robotToTagDistance =
        pose.getTranslation()
            .getDistance(
                CameraConstants.FIELD_LAYOUT.getTagPose(targetUsed).get().getTranslation());

    double xyStdDev = CameraConstants.XY_STD_DEV_COEFFICIENT * Math.pow(robotToTagDistance, 1.2);

    double thetaStdDev =
        CameraConstants.THETA_STD_DEV_COEFFICIENT * Math.pow(robotToTagDistance, 1.2);

    driveState.addVisionEstimate(
        new VisionMeasurement(
            pose.toPose2d(),
            Utils.getCurrentTimeSeconds(),
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)),
        cameraName);
  }
}
