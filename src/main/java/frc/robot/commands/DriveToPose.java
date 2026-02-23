package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class DriveToPose extends Command {
  private DrivetrainSubsystem drivetrain;
  private DriveState driveState;
  private Pose2d targetPose;
  private PIDController xControl;
  private PIDController yControl;
  private ProfiledPIDController rotControl;

  public DriveToPose(DrivetrainSubsystem subsystem, Pose2d pose) {
    drivetrain = subsystem;
    targetPose = pose;
    driveState = DriveState.getInstance();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xControl = drivetrain.getTranslationPIDController();
    yControl = drivetrain.getTranslationPIDController();
    rotControl = drivetrain.getRotationPIDController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST
            .withVelocityX(
                xControl.calculate(
                    driveState.getCurrentDriveStats().Pose.getX(), targetPose.getX()))
            .withVelocityY(
                yControl.calculate(
                    driveState.getCurrentDriveStats().Pose.getY(), targetPose.getY()))
            .withRotationalRate(
                rotControl.calculate(
                    driveState.getCurrentDriveStats().Pose.getRotation().getDegrees(),
                    targetPose.getRotation().getDegrees())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xControl.atSetpoint() && yControl.atSetpoint() && rotControl.atSetpoint();
  }
}
