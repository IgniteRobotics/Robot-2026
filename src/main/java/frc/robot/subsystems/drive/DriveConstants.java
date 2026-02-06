package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
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
}
