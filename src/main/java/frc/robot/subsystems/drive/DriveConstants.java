package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final double MAX_DRIVE_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  public static final double DEADBAND_FACTOR = 0.1;

  public static final SwerveRequest.FieldCentric DEFAULT_DRIVE_REQUEST =
      new FieldCentric()
          .withDeadband(MAX_DRIVE_SPEED * DEADBAND_FACTOR)
          .withRotationalDeadband(MAX_ANGULAR_SPEED * DEADBAND_FACTOR);

  public static final SwerveRequest.RobotCentric LEMON_HUNTING_REQUEST =
      new RobotCentric()
          .withDeadband(MAX_DRIVE_SPEED * DEADBAND_FACTOR)
          .withRotationalDeadband(MAX_ANGULAR_SPEED * DEADBAND_FACTOR);

  public static final double TRANSLATION_ALIGN_TOLERANCE = 0.01; // meters
  public static final double ROTATION_ALIGN_TOLERANCE = 0.1; // degrees

  public static final double TRANSLATION_ALIGN_KP = 1.25;
  public static final double ROTATION_ALIGN_KP = 6.28;

  public static final AngularVelocity WHEEL_RADIUS_TEST_MAX_VELOCITY = RadiansPerSecond.of(Math.PI);
  public static final AngularAcceleration WHEEL_RADIUS_TEST_RAMP_RATE =
      RadiansPerSecondPerSecond.of(0.05);
  public static final Distance DRIVETRAIN_RADIUS = Inches.of(15.365);

  public static final double SPIN_MOVE_PIVOT_CLEARANCE_METERS = 0.05;

  public static final double HUNT_SPEED = 1.5; // meters/s?
}
