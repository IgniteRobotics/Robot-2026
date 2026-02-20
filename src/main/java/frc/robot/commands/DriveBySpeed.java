// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.preferences.DoublePreference;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveBySpeed extends Command {
  private DoublePreference m_driveSpeed;
  private CommandSwerveDrivetrain m_drivetrain;

  private final SwerveRequest.RobotCentric m_driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public DriveBySpeed(CommandSwerveDrivetrain drivetrain, DoublePreference speed) {
    m_drivetrain = drivetrain;
    m_driveSpeed = speed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setControl(m_driveRequest.withVelocityX(m_driveSpeed.getValue()));
    ;
  }

  // Called once the command ends or is .interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(m_driveRequest.withVelocityX(0).withVelocityX(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
