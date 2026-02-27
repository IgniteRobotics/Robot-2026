package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class LaunchRequest {
  private Angle launchHoodTarget;
  private AngularVelocity launchVelocity;
  private AngularVelocity targetRobotAngularVelocity;
  private Rotation2d targetRobotAngle;
  private double timestamp;

  public LaunchRequest(
      Angle launchHoodTarget,
      AngularVelocity launchVelocity,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle,
      double timestamp) {
    this.launchHoodTarget =
        Rotations.of(
            launchHoodTarget.in(Degrees)
                * ShooterConstants.ROTATIONS_PER_LAUNCH_DEGREE.in(
                    Rotations)); // converts froms radians to rotations
    this.launchVelocity =
        RotationsPerSecond.of(
            launchVelocity.in(RadiansPerSecond)
                / Math.PI
                * 2); // converts from Radians Per Second to Rotations Per Second
    this.targetRobotAngularVelocity = targetRobotAngularVelocity;
    this.targetRobotAngle = targetRobotAngle;
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

  public double getTimestamp() {
    return timestamp;
  }
}
