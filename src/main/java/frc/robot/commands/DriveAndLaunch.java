// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachines.LaunchState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.LaunchRequest;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAndLaunch extends Command {

  private DrivetrainSubsystem drivetrain;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  /** Creates a new DriveAndLanuch. */
  public DriveAndLaunch(
      DrivetrainSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drivetrain = drivetrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LaunchRequest launchRequest = LaunchState.getInstance().getLaunchRequest();

    AngularVelocity turnVelocity = launchRequest.getTargetRobotAngularVelocity();

    drivetrain.applyRequest(
        () ->
            DriveConstants.DEFAULT_DRIVE_REQUEST
                .withVelocityX(
                    -1
                        * Math.copySign(
                            Math.pow(xSupplier.getAsDouble(), 2), xSupplier.getAsDouble())
                        * DriveConstants.MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
                .withVelocityY(
                    -1
                        * Math.copySign(
                            Math.pow(ySupplier.getAsDouble(), 2), ySupplier.getAsDouble())
                        * DriveConstants.MAX_DRIVE_SPEED) // Drive left with negative X (left)
                .withRotationalRate(
                    turnVelocity.in(
                        RadiansPerSecond)) // Drive counterclockwise with negative X (left)
                .withDeadband(DriveConstants.MAX_DRIVE_SPEED * 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
