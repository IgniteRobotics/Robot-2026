// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.statemachines.DriveState;
import java.util.function.Supplier;

/** Add your docs here. */
public class ParabolicLaunchRequestBuilder extends LaunchRequestBuilder {

  private ParabolicLaunchRequestBuilder instance;

  private ParabolicLaunchRequestBuilder(Supplier<Pose3d> targetPose) {
    super(targetPose);
  }

  @Override
  public ParabolicLaunchRequestBuilder getInstance(Supplier<Pose3d> targetPose) {
    if (this.instance != null) {
      instance = new ParabolicLaunchRequestBuilder(targetPose);
      return instance;
    } else {
      this.targetPose = targetPose;
      return instance;
    }
  }

  @Override
  public LaunchRequest createLaunchRequest() {
    double y1 = ShooterConstants.SHOOTER_HEIGHT.in(Meters);
    double x2 =
        DriveState.getInstance()
            .getCurrentDriveStats()
            .Pose
            .getTranslation()
            .getDistance(targetPose.get().getTranslation().toTranslation2d());
    // double y2 = ShooterConstants.GOAL_HEIGHT.in(Meters);
    double y2 = targetPose.get().getMeasureZ().in(Meters);

    double slope = ShooterConstants.OPTIMAL_ENTRY_SLOPE;
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
    } while ((vertex > x2 - ShooterConstants.MIN_VERTEX_DISTANCE.in(Meters))
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

    return new LaunchRequest(
        theta,
        angularVelocity,
        LinearVelocity.ofBaseUnits(0, MetersPerSecond),
        DriveState.getInstance().getCurrentDriveStats().Pose.getRotation());
  }
}
