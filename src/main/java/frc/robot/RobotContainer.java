// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveBySpeed;
import frc.robot.commands.WheelSlipTest;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivePreferences;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

@Logged
public class RobotContainer {

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.MAX_DRIVE_SPEED);

  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  @Logged(name = "Intake")
  public final IntakeSubsystem intake = new IntakeSubsystem();

  @Logged(name = "Indexer")
  public final IndexerSubsystem indexer = new IndexerSubsystem();

  @Logged(name = "Climber")
  public final ClimberSubsystem climber = new ClimberSubsystem();

  @Logged(name = "Shooter")
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Seed", drivetrain.runOnce(drivetrain::seedFieldCentric));
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
                            * Math.copySign(
                                Math.pow(driverJoystick.getLeftY(), 2), driverJoystick.getLeftY())
                            * DriveConstants
                                .MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -1
                            * Math.copySign(
                                Math.pow(driverJoystick.getLeftX(), 2), driverJoystick.getLeftX())
                            * DriveConstants.MAX_DRIVE_SPEED) // Drive left with negative X (left)
                    .withRotationalRate(
                        -1
                            * Math.copySign(
                                Math.pow(driverJoystick.getRightX(), 2), driverJoystick.getRightX())
                            * DriveConstants
                                .MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X (left)
                    .withDeadband(DriveConstants.MAX_DRIVE_SPEED * 0.1)
                    .withRotationalDeadband(DriveConstants.MAX_ANGULAR_SPEED * 0.1)));
  }

  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverJoystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    // joystick.x().onTrue(drivetrain.sysIdSteer());
    // joystick.y().onTrue(drivetrain.sysIdTranslation());
    driverJoystick.x().onTrue(new WheelSlipTest(drivetrain));
    driverJoystick
        .y()
        .whileTrue(new DriveBySpeed(drivetrain, DrivePreferences.onemeter_speed)); // Testing only

    driverJoystick
        .rightTrigger()
        .onTrue(intake.setExtendNoPID())
        .onFalse(intake.stopIntakeNoPID());

    driverJoystick
        .leftTrigger()
        .onTrue(intake.setRetractNoPID())
        .onFalse(intake.stopIntakeNoPID());

    driverJoystick
        .a()
        .whileTrue(intake.setRollerNoPID().repeatedly())
        .onFalse(intake.stopRollerNoPID());

    operatorJoystick
        .leftTrigger()
        .whileTrue(shooter.launchLemonsCommandNoPID().repeatedly())
        .onFalse(shooter.stopLaunchLemonsNoPIDCommand());

    operatorJoystick
        .rightTrigger()
        .whileTrue(indexer.setIndexerNoPID().repeatedly())
        .onFalse(indexer.stopIndexerNoPID());

    // Reset the field-centric heading on left bumper press.
    driverJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
