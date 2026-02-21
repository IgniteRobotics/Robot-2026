// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public interface LaunchRequestBuilder {

  public record LaunchRequest(
      Angle launchHoodTarget,
      AngularVelocity launchVelocity,
      LinearVelocity driveVelocity,
      Rotation2d driveAngle) {}

  public LaunchRequest createLaunchRequest();
}
