package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.DriveState;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  private DriveState driveState = DriveState.getInstance();
  private SwerveDriveState driveStats;

  public class VisionMeasurement {
    private EstimatedRobotPose estimatedPose;
    private double timestamp;
    private Vector<N3> trustValues;

    public VisionMeasurement(EstimatedRobotPose pose, double time, Vector<N3> trust) {
      estimatedPose = pose;
      timestamp = time;
      trustValues = trust;
    }

    public EstimatedRobotPose getEstimatedPose() {
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

  public void periodic() {
    if (driveState.hasDriveStats()) {
      // this makes sure that the different parts of the periodic use different stats
      driveStats = driveState.getCurrentDriveStats();
      // camera1
      // This is FIFO, so the oldest is given first and the newest last'
      CameraConstants.photonPoseEstimator1.setReferencePose(driveStats.Pose);
      var camera1Results = CameraConstants.photonCamera1.getAllUnreadResults();
      for (var result : camera1Results) {
        Optional<EstimatedRobotPose> estimatedPose =
            CameraConstants.photonPoseEstimator1.update(result);
        if (!estimatedPose.isEmpty()) {
          evaluateMeasurement(estimatedPose.get(), CameraConstants.photonCameraName1);
        }
      }

      // camera 2
      CameraConstants.photonPoseEstimator2.setReferencePose(driveStats.Pose);
      var camera2Results = CameraConstants.photonCamera2.getAllUnreadResults();
      for (var result : camera2Results) {
        Optional<EstimatedRobotPose> estimatedPose =
            CameraConstants.photonPoseEstimator2.update(result);
        if (!estimatedPose.isEmpty()) {
          evaluateMeasurement(estimatedPose.get(), CameraConstants.photonCameraName2);
        }
      }
    }
  }

  public void setPipeline(int index) {
    CameraConstants.photonCamera1.setPipelineIndex(index);
    CameraConstants.photonCamera1.setPipelineIndex(index);
  }

  // evaluates the estimations before export
  private void evaluateMeasurement(EstimatedRobotPose pose, String cameraName) {
    // default trust values (NOTE: higher values means less trust)
    // the trust values are equivalent to standard deviations
    double xyStds = 0.5;
    double thetaStd = 0.5;

    // "maximum value" variable instatiation
    double maxTargetArea = 0;
    double highestAmbiguity = 0;

    // iterates through every target (AprilTag) used to calculate the pose
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      if (target.getPoseAmbiguity() > highestAmbiguity) {
        highestAmbiguity = target.getPoseAmbiguity();
      }
      if (target.area > maxTargetArea) {
        maxTargetArea = target.area;
      }
    }

    // if the pose has a target with too much ambiguity, don't use it
    if (highestAmbiguity > CameraConstants.MAXIMUM_ALLOWED_AMBIGUITY) return;

    // precalculated value (distance from past robot pose to estimated pose)
    double poseDistance =
        pose.estimatedPose
            .toPose2d()
            .getTranslation()
            .getDistance(driveStats.Pose.getTranslation());

    // if the target is large
    // at 2m, the target is .5% of the image.
    // at 1m, the target is 2.5% of the image.
    if (maxTargetArea > 2) {
      // we're not moving, trust the pose
      if (driveStats.Speeds.vxMetersPerSecond + driveStats.Speeds.vyMetersPerSecond < 0.2) {
        xyStds = 0.05;
        if (driveStats.Speeds.omegaRadiansPerSecond < 0.1) {
          thetaStd = 0.05;
        }
        // new pose is close to the old pose, trust the pose
      } else if (poseDistance < 0.5) {
        xyStds = 0.15;
        thetaStd = 0.15;
      }
    } else if (maxTargetArea > 0.5) {
      // we're not moving, trust the pose
      if (driveStats.Speeds.vxMetersPerSecond + driveStats.Speeds.vyMetersPerSecond < 0.2) {
        xyStds = 0.1;
        thetaStd = 0.1;
        if (driveStats.Speeds.omegaRadiansPerSecond < 0.1) {
          thetaStd = 0.1;
        }
        // new pose is close to the old pose, trust the pose
      } else if (poseDistance < 0.5) {
        xyStds = 0.25;
        thetaStd = 0.25;
      }
      // if targets are faraway, we need to have a smaller ambiguity
    } else if (highestAmbiguity > 0.1) {
      xyStds = 0.75;
      thetaStd = 0.75;
    }

    // if you're spinning, bail.
    if (driveStats.Speeds.omegaRadiansPerSecond > Math.PI) {
      return;
    } else if (driveStats.Speeds.omegaRadiansPerSecond
        > 0.5) { // if you're rotating quickly, trust the pose less
      thetaStd = 0.75;
    }

    // uploads the pose with its trust values to the drive statemachine
    // this pose will then be exported to the drivetrain to help navigation
    // note that we use a singular trust value for both the x and y trust values
    driveState.addVisionEstimate(
        new VisionMeasurement(
            pose, Utils.getCurrentTimeSeconds(), VecBuilder.fill(xyStds, xyStds, thetaStd)),
        cameraName);
  }
}
