// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class MappedLaunchRequestBuilder implements LaunchRequestBuilder {

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;

  // Launching Maps
  //   private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
  //   new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  // TODO: All of this is made up.  Need real numbers.
  static {
    minDistance = 0.9;
    maxDistance = 4.0;
    passingMinDistance = 0.0;
    passingMaxDistance = 100000;

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

    passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(3.0));
    passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(6.1));

    passingFlywheelSpeedMap.put(passingMinDistance, 70.0);
    passingFlywheelSpeedMap.put(passingMaxDistance, 100.0);
  }

  public LaunchRequest createLaunchRequest(
      boolean passing,
      double distance,
      AngularVelocity targetRobotAngularVelocity,
      Rotation2d targetRobotAngle,
      Distance targetDistance) {

    // calculate hood angle
    double hoodAngle =
        passing ? passingHoodAngleMap.get(distance).getRotations() : hoodAngleMap.get(distance);

    // calculate flywheel speed
    double flywheelSpeed =
        passing ? passingFlywheelSpeedMap.get(distance) : flywheelSpeedMap.get(distance);

    return new LaunchRequest(
        Rotations.of(hoodAngle),
        RotationsPerSecond.of(flywheelSpeed),
        targetRobotAngularVelocity,
        targetRobotAngle,
        targetDistance,
        Utils.getCurrentTimeSeconds());
  }
}
