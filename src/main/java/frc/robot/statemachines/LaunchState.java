package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.ShooterConstants;

public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  AllianceState allianceState = AllianceState.getInstance();
  private LaunchCalculator launchCalculator = LaunchCalculator.getInstance();

  @Logged(name = "Current Launch Reqeust")
  private LaunchRequest currentLaunchRequest = null;

  @Logged(name = "3D Target Pose")
  private Pose3d targetPose3d = ShooterConstants.RED_TARGET;

  @Logged(name = "2D Target Pose")
  private Pose2d targetPose2d = ShooterConstants.RED_TARGET.toPose2d();

  private LaunchType builderType = LaunchType.MAPPED;

  public LaunchRequest getLaunchRequest() {
    return currentLaunchRequest;
  }

  public void refreshRequest() {
    currentLaunchRequest = launchCalculator.refreshRequest(targetPose3d, builderType);

    SmartDashboard.putNumber(
        "Launch/Target Robot Angle (Degrees)",
        currentLaunchRequest.getTargetRobotAngle().getDegrees());
    SmartDashboard.putNumber(
        "Launch/Target Robot Angular Velocity (Rotations Per Second)",
        currentLaunchRequest.getTargetRobotAngularVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Launch/Flywheel Velocity (Rotations Per Second)",
        currentLaunchRequest.getFlywheelVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber(
        "Launch/Hood Target (Rotations)", currentLaunchRequest.getHoodTarget().in(Rotations));

    Alliance alliance = allianceState.getAlliance();
    if (alliance == Alliance.Red) {
      setTargetPose3d(ShooterConstants.RED_TARGET);
      setTargetPose2d(ShooterConstants.RED_TARGET.toPose2d());
    } else {
      setTargetPose3d(ShooterConstants.BLUE_TARGET);
      setTargetPose2d(ShooterConstants.BLUE_TARGET.toPose2d());
    }
  }

  public void setTargetPose3d(Pose3d target) {
    this.targetPose3d = target;
  }

  public void setTargetPose2d(Pose2d target) {
    this.targetPose2d = target;
  }

  public void setBuilderType(LaunchType builderType) {
    this.builderType = builderType;
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
