// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class MappedLaunchRequestBuilder implements LaunchRequestBuilder {

  private static final double minDistance;
  private static final double maxDistance;

  // Launching Maps
  //   private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
  //   new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  // TODO: All of this is made up.  Need real numbers.
  static {
    minDistance = 0.9;
    maxDistance = 4.0;

    hoodAngleMap.put(0.99, 0.0);
    hoodAngleMap.put(1.62, 1.0);
    hoodAngleMap.put(1.94, 2.1);
    hoodAngleMap.put(2.53, 3.3);
    hoodAngleMap.put(3.00, 4.0);
    hoodAngleMap.put(3.51, 4.0);
    hoodAngleMap.put(6.00, 6.1); // put in a value to max out the hood

    flywheelSpeedMap.put(0.99, 57.7);
    flywheelSpeedMap.put(1.62, 64.3);
    flywheelSpeedMap.put(1.94, 64.7);
    flywheelSpeedMap.put(2.53, 70.8);
    flywheelSpeedMap.put(3.00, 74.5);
    flywheelSpeedMap.put(3.51, 80.0);
    flywheelSpeedMap.put(6.00, 108.0); // put in a value to max out the hood
  }

  public LaunchRequest createLaunchRequest(
      boolean passing,
      double distance,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle,
      Distance targetDistance) {

    double hoodAngle, flywheelSpeed;
    if (passing) {
      hoodAngle = 6.1;
      flywheelSpeed = ShooterPreferences.passingFlywheelSpeed.getValue();
    } else {
      // calculate hood angle
      hoodAngle = hoodAngleMap.get(distance);
      // calculate flywheel speed
      flywheelSpeed = flywheelSpeedMap.get(distance);
    }

    return new LaunchRequest(
        Rotations.of(hoodAngle),
        RotationsPerSecond.of(flywheelSpeed),
        targetRobotAngularVelocity,
        targetRobotAngle,
        targetDistance,
        Utils.getCurrentTimeSeconds());
  }
}
