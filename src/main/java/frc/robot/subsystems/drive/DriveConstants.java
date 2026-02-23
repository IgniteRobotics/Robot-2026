package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
  public static final double MAX_DRIVE_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  public static final double DEADBAND_FACTOR = 0.1;

  public static final SwerveRequest.FieldCentric DEFAULT_DRIVE_REQUEST =
      new FieldCentric()
          .withDeadband(MAX_DRIVE_SPEED * DEADBAND_FACTOR)
          .withRotationalDeadband(MAX_ANGULAR_SPEED * DEADBAND_FACTOR)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final SwerveRequest.FieldCentric AUTO_DRIVE_REQUEST =
      new FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity) // Closed Loop Translation
          .withSteerRequestType(SteerRequestType.Position) // Closed Loop Steer (Default)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // Prevents "flips"

  public static final double TRANSLATION_ALIGN_TOLERANCE = 0.01; // meters
  public static final double ROTATION_ALIGN_TOLERANCE = 1; // degrees

  public static final double TRANSLATION_ALIGN_KP = 0.0001;
  public static final double ROTATION_ALIGN_KP = 6.28;

  // ***** constants for drive to pose / autopilot *****
  public static final LinearVelocity AP_MAX_VELOCITY =
      LinearVelocity.ofBaseUnits(4.5, MetersPerSecond);
  public static final LinearAcceleration AP_MAX_ACCELERATION =
      LinearAcceleration.ofRelativeUnits(3.5, MetersPerSecondPerSecond);

  public static final AngularVelocity AP_MAX_ANGULAR_VELOCITY =
      AngularVelocity.ofBaseUnits(1.0, RotationsPerSecond);
  public static final AngularAcceleration AP_MAX_ANGULAR_ACCELERATION =
      AngularAcceleration.ofBaseUnits(2, RotationsPerSecondPerSecond);

  // error at which feed forward is cut off completely.
  public static final Distance AP_LINEAR_FF_MIN_ERROR = Distance.ofBaseUnits(1, Centimeter);
  public static final Angle AP_ROTATION_FF_MIN_ERROR = Angle.ofBaseUnits(1, Degree);

  // error at which 100% of feed forward is applied
  public static final Distance AP_LINEAR_FF_MAX_ERROR = Distance.ofBaseUnits(5, Centimeter);
  public static final Angle AP_ROTATION_FF_MAX_ERROR = Angle.ofBaseUnits(5, Degree);

  public static final LinearVelocity AP_MIN_APPLIED_VELOCITY =
      LinearVelocity.ofBaseUnits(0.5, MetersPerSecond);
}
