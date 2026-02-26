package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.statemachines.LaunchCalculator.LaunchRequest;

public interface LaunchRequestBuilder {
  public LaunchRequest createLaunchRequest(
      boolean passing,
      double distance,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle);
}
