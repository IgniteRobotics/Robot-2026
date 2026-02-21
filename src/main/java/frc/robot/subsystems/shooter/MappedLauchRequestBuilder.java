// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.statemachines.DriveState;
import java.util.function.Supplier;

/** Add your docs here. */
public class MappedLauchRequestBuilder implements LaunchRequestBuilder {

  private Supplier<Pose3d> targetPose;

  private LaunchRequest currentLaunchRequest;

  public MappedLauchRequestBuilder(Supplier<Pose3d> targetPose) {
    this.targetPose = targetPose;
  }

  private double loopPeriodSecs = 0.02;

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.8 / loopPeriodSecs));

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // TODO: All of this is made up.  Need real numbers.
  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    passingMinDistance = 0.0;
    passingMaxDistance = 100000;
    phaseDelay = 0.03;

    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    flywheelSpeedMap.put(1.34, 210.0);
    flywheelSpeedMap.put(1.78, 220.0);
    flywheelSpeedMap.put(2.17, 220.0);
    flywheelSpeedMap.put(2.81, 230.0);
    flywheelSpeedMap.put(3.82, 250.0);
    flywheelSpeedMap.put(4.09, 255.0);
    flywheelSpeedMap.put(4.40, 260.0);
    flywheelSpeedMap.put(4.77, 265.0);
    flywheelSpeedMap.put(5.57, 275.0);
    flywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(0.0));
    passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(0.0));

    passingFlywheelSpeedMap.put(passingMinDistance, 0.0);
    passingFlywheelSpeedMap.put(passingMaxDistance, 0.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
  }

  public LaunchRequest createLaunchRequest() {

    boolean passing = targetPose.get().getZ() == 0.0; // target pose is the floor, so we're passing.

    // current pose and movement
    Pose2d estimatedRobotPose = DriveState.getInstance().getCurrentDriveStats().Pose;
    ChassisSpeeds robotSpeeds = DriveState.getInstance().getCurrentDriveStats().Speeds;

    // predicted pose
    estimatedRobotPose.exp(
        new Twist2d(
            robotSpeeds.vxMetersPerSecond * phaseDelay,
            robotSpeeds.vyMetersPerSecond * phaseDelay,
            robotSpeeds.omegaRadiansPerSecond * phaseDelay));

    // TODO:  for now assume they're the same.  calculate offsets later
    Pose2d launcherPose = estimatedRobotPose;
    double launcherToTargetDistance =
        launcherPose.getTranslation().getDistance(targetPose.get().toPose2d().getTranslation());

    // Calculate field relative velocity
    double xVelocity = DriveState.getInstance().getFieldVelocity().vxMetersPerSecond;
    double yVelocity = DriveState.getInstance().getFieldVelocity().vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPose;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = xVelocity * timeOfFlight;
      double offsetY = yVelocity * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPose.getRotation());
      lookaheadLauncherToTargetDistance =
          targetPose
              .get()
              .getTranslation()
              .toTranslation2d()
              .getDistance(lookaheadPose.getTranslation());
    }

    // calcuate rotation angle
    Rotation2d driveAngle =
        getDriveAngle(lookaheadPose, targetPose.get().getTranslation().toTranslation2d());
    // if no last drive angle, default to the robot's current pose info.
    if (lastDriveAngle == null) lastDriveAngle = estimatedRobotPose.getRotation();

    // calculate hood angle
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();

    // calculate flywheel speed
    double flywheelSpeed =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // filter for smoothing
    double driveVelocity =
        driveAngleFilter.calculate(driveAngle.minus(lastDriveAngle).getRadians() / loopPeriodSecs);
    lastDriveAngle = driveAngle;

    currentLaunchRequest =
        new LaunchRequest(
            Angle.ofBaseUnits(hoodAngle, Radians),
            AngularVelocity.ofBaseUnits(flywheelSpeed, RadiansPerSecond),
            LinearVelocity.ofBaseUnits(driveVelocity, MetersPerSecond),
            driveAngle);

    return currentLaunchRequest;
  }

  private static Rotation2d getDriveAngle(Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotPose.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle = fieldToHubAngle.plus(hubAngle).plus(robotPose.getRotation());
    return driveAngle;
  }
}
