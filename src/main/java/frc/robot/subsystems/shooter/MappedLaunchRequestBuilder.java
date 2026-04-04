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

  // Launching Maps
  //   private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
  //   new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(1.3, 0.0);
    hoodAngleMap.put(2.0, 0.6);
    hoodAngleMap.put(2.6, 1.4);
    hoodAngleMap.put(3.1, 1.8);
    hoodAngleMap.put(3.21, 2.2);
    hoodAngleMap.put(3.7, 2.6);
    hoodAngleMap.put(4.2, 2.6);
    hoodAngleMap.put(5.4, 3.0);

    flywheelSpeedMap.put(1.3, 326.0);
    flywheelSpeedMap.put(2.0, 342.6);
    flywheelSpeedMap.put(2.6, 359.0);
    flywheelSpeedMap.put(3.1, 391.9);
    flywheelSpeedMap.put(3.21, 391.0);
    flywheelSpeedMap.put(3.7, 408.4);
    flywheelSpeedMap.put(4.2, 424.8);
    flywheelSpeedMap.put(5.4, 490.6);

    /* Old shooter settings
    hoodAngleMap.put(0.99, 0.0);
    hoodAngleMap.put(1.62, 1.0);
    hoodAngleMap.put(1.94, 2.1);
    hoodAngleMap.put(2.53, 3.3);
    hoodAngleMap.put(3.00, 4.0);
    hoodAngleMap.put(3.51, 4.0);
    hoodAngleMap.put(6.00, 6.1); 

    flywheelSpeedMap.put(0.99, 57.7);
    flywheelSpeedMap.put(1.62, 64.3);
    flywheelSpeedMap.put(1.94, 64.7);
    flywheelSpeedMap.put(3.00, 74.5);
    flywheelSpeedMap.put(3.51, 80.0);
    flywheelSpeedMap.put(6.00, 108.0); 
    */
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
