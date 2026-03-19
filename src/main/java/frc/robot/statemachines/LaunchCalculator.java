package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.statemachines.LaunchState.LaunchType;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.MappedLaunchRequestBuilder;
import frc.robot.subsystems.shooter.ParabolicLaunchRequestBuilder;

public class LaunchCalculator {

  private static LaunchCalculator single_instance = null;

  private LaunchCalculator() {}

  protected static synchronized LaunchCalculator getInstance() {
    if (single_instance == null) single_instance = new LaunchCalculator();
    return single_instance;
  }

  private double loopPeriodSecs = 0.02;

  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.8 / loopPeriodSecs));

  private static final double phaseDelay;

  private static final InterpolatingDoubleTreeMap hubTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // TODO: All of this is made up.  Need real numbers.
  static {
    phaseDelay = 0.03;

    hubTimeOfFlightMap.put(5.68, 1.16);
    hubTimeOfFlightMap.put(4.55, 1.12);
    hubTimeOfFlightMap.put(3.15, 1.11);
    hubTimeOfFlightMap.put(1.88, 1.09);
    hubTimeOfFlightMap.put(1.38, 0.90);

    passingTimeOfFlightMap.put(6.68, 1.16);
    passingTimeOfFlightMap.put(5.55, 1.12);
    passingTimeOfFlightMap.put(4.15, 1.11);
    passingTimeOfFlightMap.put(2.88, 1.09);
    passingTimeOfFlightMap.put(2.38, 0.90);
  }

  protected LaunchRequest refreshRequest(Pose3d target, LaunchType builderType) {

    boolean passing = target.getZ() < 0.1;

    // current pose and movement
    Pose2d estimatedRobotPose = DriveState.getInstance().getCurrentDriveStats().Pose;
    ChassisSpeeds robotSpeeds = DriveState.getInstance().getCurrentDriveStats().Speeds;

    // predicted pose
    estimatedRobotPose =
        estimatedRobotPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * phaseDelay,
                robotSpeeds.vyMetersPerSecond * phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * phaseDelay));

    // TODO:  for now assume they're the same.  calculate offsets later
    Pose2d launcherPose = estimatedRobotPose;
    double launcherToTargetDistance =
        launcherPose.getTranslation().getDistance(target.toPose2d().getTranslation());

    // Calculate field relative velocity
    double xSpeed = DriveState.getInstance().getFieldVelocity().vxMetersPerSecond;
    double ySpeed = DriveState.getInstance().getFieldVelocity().vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : hubTimeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPose;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : hubTimeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = xSpeed * timeOfFlight;
      double offsetY = ySpeed * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPose.getRotation());
      lookaheadLauncherToTargetDistance =
          target.getTranslation().toTranslation2d().getDistance(lookaheadPose.getTranslation());
    }

    SmartDashboard.putNumber("Launch Request/Look Ahead Pose/X", lookaheadPose.getX());
    SmartDashboard.putNumber("Launch Request/Look Ahead Pose/Y", lookaheadPose.getY());
    SmartDashboard.putNumber(
        "Launch Request/Look Ahead Target Distance", lookaheadLauncherToTargetDistance);

    // calcuate rotation angle
    Rotation2d targetRobotAngle =
        target.getTranslation().toTranslation2d().minus(lookaheadPose.getTranslation()).getAngle();

    // Rotation2d targetRobotAngle = getDriveAngle(lookaheadPose,
    // target.getTranslation().toTranslation2d());

    AngularVelocity targetRobotAngularVelocity =
        // RadiansPerSecond.of(
        //     driveAngleFilter.calculate(
        //         targetRobotAngle
        //
        // .minus(DriveState.getInstance().getPreviousDriveStats().Pose.getRotation())
        //                 .getRadians()
        //             / loopPeriodSecs));
        RadiansPerSecond.of(
            targetRobotAngle
                .minus(DriveState.getInstance().getPreviousDriveStats().Pose.getRotation())
                .getRadians());

    if (builderType == LaunchType.MAPPED)
      return new MappedLaunchRequestBuilder()
          .createLaunchRequest(
              passing,
              lookaheadLauncherToTargetDistance,
              targetRobotAngularVelocity,
              targetRobotAngle,
              Meters.of(launcherToTargetDistance));
    else
      return new ParabolicLaunchRequestBuilder()
          .createLaunchRequest(
              passing,
              lookaheadLauncherToTargetDistance,
              targetRobotAngularVelocity,
              targetRobotAngle,
              Meters.of(launcherToTargetDistance));
  }

  /*
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
  */
}
