// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.WheelSlipTest;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class RobotContainer {

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_DRIVE_SPEED);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureSubsystemDefaultCommands();
    configureBindings();
  }

  private void configureSubsystemDefaultCommands() {

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                DriveConstants.DEFAULT_DRIVE_REQUEST
                    .withVelocityX(
                        -1
                            * Math.copySign(Math.pow(joystick.getLeftY(), 2), joystick.getLeftY())
                            * DriveConstants
                                .MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -1
                            * Math.copySign(Math.pow(joystick.getLeftX(), 2), joystick.getLeftX())
                            * DriveConstants.MAX_DRIVE_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(
                        -1
                            * Math.copySign(Math.pow(joystick.getRightX(), 2), joystick.getRightX())
                            * DriveConstants
                                .MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
            ));
  }

  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    joystick.x().onTrue(drivetrain.sysIdSteer());
    joystick.y().onTrue(drivetrain.sysIdTranslation());
    joystick.b().whileTrue(new WheelSlipTest(drivetrain)); // Testing only

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
