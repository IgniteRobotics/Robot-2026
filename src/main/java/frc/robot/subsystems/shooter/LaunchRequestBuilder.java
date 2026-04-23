package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public interface LaunchRequestBuilder {
  public LaunchRequest createLaunchRequest(
      boolean passing, double distance, Rotation2d targetRobotAngle, Distance targetDistance);
}
