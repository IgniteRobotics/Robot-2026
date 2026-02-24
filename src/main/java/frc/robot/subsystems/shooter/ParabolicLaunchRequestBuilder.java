// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.statemachines.LaunchCalculator.LaunchRequest;

/** Add your docs here. */
public class ParabolicLaunchRequestBuilder implements LaunchRequestBuilder{

  public LaunchRequest createLaunchRequest(
      boolean passing,
      double distance,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle) {
    double y1 = ShooterConstants.SHOOTER_HEIGHT.in(Meters);
    double x2 = distance;
    double y2 = passing ? 0 : ShooterConstants.GOAL_HEIGHT.in(Meters);

    double slope;
    if (passing) slope = ShooterConstants.OPTIMAL_PASSING_ENTRY_SLOPE;
    else slope = ShooterConstants.OPTIMAL_HUB_ENTRY_SLOPE;

    double a, b, vertex;
    Angle theta, motorAngle;
    do {
      // system of equations
      // (y2) = a(x2*x2) + b(x2) + y1
      // slope = 2a(x2) + b
      a = (slope * x2 + y1 - y2) / (x2 * x2);
      b = (slope - 2 * a * x2);
      theta = Radians.of(Math.atan(b)); // launch angle (Hood Angle Conversion: MATH.PI/2 - theta)
      motorAngle = Radians.of(Math.PI / 2 - theta.in(Radians));
      vertex = -1 * b / (2 * a);
      slope -= 0.05;
    } while ((passing || vertex > x2 - ShooterConstants.MIN_HUB_VERTEX_DISTANCE.in(Meters))
        && !(motorAngle.in(Degrees) < ShooterConstants.MIN_HOOD_ANGLE.in(Degrees)));

    if (motorAngle.in(Degrees) < ShooterConstants.MIN_HOOD_ANGLE.in(Degrees)) return null;

    // system of equations
    // (-b/2a) = (velocity)*cos(theta)*t
    // 2g(t) = (velocity)*sin(theta)
    LinearVelocity velocity =
        MetersPerSecond.of(
            Math.sqrt(
                2 * 9.8 * vertex / (Math.sin(theta.in(Radians)) * Math.cos(theta.in(Radians)))));

    // Calculate angular velocity using the formula: omega = v / r
    double angularVelocityMagnitude =
        velocity.in(MetersPerSecond) / ShooterConstants.FLYWHEEL_RADIUS.in(Meters);

    // Create a typed AngularVelocity object (optional, for use within the units library ecosystem)
    AngularVelocity angularVelocity = RadiansPerSecond.of(angularVelocityMagnitude);

    return new LaunchRequest(theta, angularVelocity, targetRobotAngularVelocity, targetRobotAngle);
  }
}
