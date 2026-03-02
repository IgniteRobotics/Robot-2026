package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
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

}
