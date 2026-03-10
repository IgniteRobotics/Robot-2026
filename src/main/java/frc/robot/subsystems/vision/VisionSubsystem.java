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
        createMeasurement(result, CameraConstants.photonCameraName_Front);

      // Left Camera
      var leftCameraResults = CameraConstants.photonCamera_Left.getAllUnreadResults();
      for (var result : leftCameraResults)
        createMeasurement(result, CameraConstants.photonCameraName_Left);

      // Right Camera
      var rightCameraResults = CameraConstants.photonCamera_Right.getAllUnreadResults();
      for (var result : rightCameraResults)
        createMeasurement(result, CameraConstants.photonCameraName_Right);
    }
  }

  // code from PoseEstimator class
  private void createMeasurement(PhotonPipelineResult result, String cameraName) {
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
    if (lowestAmbiguityTarget == null
        || lowestAmbiguityScore > CameraConstants.MAXIMUM_ALLOWED_AMBIGUITY) return;

    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Pose3d targetPosition = CameraConstants.FIELD_LAYOUT.getTagPose(targetFiducialId).get();

    evaluateMeasurement(
        targetPosition
            .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
            .transformBy(CameraConstants.cameraTransformMap.get(cameraName).inverse()),
        lowestAmbiguityTarget,
        cameraName);
  }

  // evaluates the estimations before export
  private void evaluateMeasurement(Pose3d pose, PhotonTrackedTarget targetUsed, String cameraName) {

    double estimateDistance =
        driveStats.Pose.getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    double robotToTagDistance =
        pose.getTranslation()
            .getDistance(
                CameraConstants.FIELD_LAYOUT
                    .getTagPose(targetUsed.getFiducialId())
                    .get()
                    .getTranslation());

    if (!RobotModeTriggers.disabled().getAsBoolean()
        && estimateDistance > TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02) return;

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
