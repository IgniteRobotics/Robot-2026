// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@Logged
public class WheelSlipTest extends Command {
  private DrivetrainSubsystem m_drive;
  private double step;
  private ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
  private SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /** Creates a new WheelSlipTest. */
  public WheelSlipTest(DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  public void incrementStep() {
    step += 0.025;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drive.seedFieldCentric();
    point.withModuleDirection(new Rotation2d(0.0));
    step = 0;
    scheduler.scheduleAtFixedRate(
        () -> {
          incrementStep();
        },
        3,
        1,
        TimeUnit.SECONDS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setDrivePower(step);
    point.withModuleDirection(new Rotation2d(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Test has ended");
    m_drive.printDriveMotorCurrents();
    m_drive.setDrivePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
