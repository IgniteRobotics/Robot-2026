package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

@Logged
public class LaunchRequest {
  @Logged(name = "Launch Hood Angle", importance = Logged.Importance.CRITICAL)
  private Angle launchHoodTarget;

  @Logged(name = "Launch Flywheel Velocity", importance = Logged.Importance.CRITICAL)
  private AngularVelocity launchVelocity;

  @Logged(name = "Target Robot Angular Velocity", importance = Logged.Importance.CRITICAL)
  private AngularVelocity targetRobotAngularVelocity;

  @Logged(name = "Target Robot Angle", importance = Logged.Importance.CRITICAL)
  private Rotation2d targetRobotAngle;

  @Logged(name = "Robot distance to Target", importance = Logged.Importance.CRITICAL)
  private Distance targetDistance;

  private double timestamp;

  public LaunchRequest(
      Angle launchHoodTarget,
      AngularVelocity launchVelocity,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle,
      Distance targetDistance,
      double timestamp) {
    this.launchHoodTarget = launchHoodTarget;
    this.launchVelocity = launchVelocity;
    this.targetRobotAngularVelocity = targetRobotAngularVelocity;
    this.targetRobotAngle = targetRobotAngle;
    this.targetDistance = targetDistance;
    this.timestamp = timestamp;
  }

  public Angle getHoodTarget() {
    return launchHoodTarget;
  }

  public AngularVelocity getFlywheelVelocity() {
    return launchVelocity;
  }

  public AngularVelocity getTargetRobotAngularVelocity() {
    return targetRobotAngularVelocity;
  }

  public Rotation2d getTargetRobotAngle() {
    return targetRobotAngle;
  }

  public Distance getTargetDistance() {
    return targetDistance;
  }

  public double getTimestamp() {
    return timestamp;
  }
}
