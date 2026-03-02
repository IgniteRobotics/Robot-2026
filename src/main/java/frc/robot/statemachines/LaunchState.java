package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
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

  private LaunchCalculator launchCalculator = LaunchCalculator.getInstance();
  private LaunchRequest currentLaunchRequest = null;

  private Pose3d targetPose3d = ShooterConstants.RED_TARGET;
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
  }

  public void setTargetPose3d(Pose3d target) {
    this.targetPose3d = target;
  }

  public void setBuilderType(LaunchType builderType) {
    this.builderType = builderType;
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
