// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.statemachines.DriveState;
import frc.robot.statemachines.LaunchState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivePreferences;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePreferences;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.ui.UISubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

@Logged
public class RobotContainer {
  private final Telemetry logger = new Telemetry(DriveConstants.MAX_DRIVE_SPEED);

  private final CommandXboxController driverJoystick = new CommandXboxController(0);
  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  @Logged(name = "Intake")
  public final IntakeSubsystem intake = new IntakeSubsystem();

  @Logged(name = "Indexer")
  public final IndexerSubsystem indexer = new IndexerSubsystem();

  @Logged(name = "Shooter")
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  @Logged(name = "Climber")
  public final ClimberSubsystem climber = new ClimberSubsystem();

  @Logged(name = "Vision")
  public final VisionSubsystem vision = new VisionSubsystem();

  @Logged(name = "UI Feedback")
  public final UISubsystem uiFeedback =
      new UISubsystem(driverJoystick.getHID(), operatorJoystick.getHID());

  private final DriveState driveState = DriveState.getInstance();
  private final LaunchState launchState = LaunchState.getInstance();

  private final Command driveAndLaunchCommand =
      drivetrain
          .applyRequest(() -> getDriveAndLaunchRequest())
          // .alongWith(shooter.spinFlywheelCommand());
          .alongWith(shooter.spinFlywheelRanged());

  private final Command autonShootCommand =
      drivetrain
          .applyRequest(() -> getDriveAndLaunchRequest())
          // .alongWith(shooter.spinFlywheelCommand());
          .alongWith(shooter.spinFlywheelRanged())
          .alongWith(new WaitCommand(1).andThen(indexer.pulsingIndexCommand()));

  private final Command autonShootCommandHard_Coded =
      shooter
          .spinFlywheelHardCoded()
          .alongWith(new WaitCommand(1).andThen(indexer.pulsingIndexCommand()));

  private final Command stopShotCommand =
      indexer
          .stopFullIndexingNoPID()
          .andThen(shooter.stopFlywheelCommand())
          .andThen(shooter.stowHood());

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Seed", drivetrain.runOnce(drivetrain::seedFieldCentric));
    NamedCommands.registerCommand("AutonShoot", autonShootCommand);
    NamedCommands.registerCommand("AutonShootHardCoded", autonShootCommandHard_Coded);
    NamedCommands.registerCommand("StopShot", stopShotCommand);
    NamedCommands.registerCommand("Collect Intake", intake.collectNoPIDCommand());
    NamedCommands.registerCommand("Stow Intake", intake.stowNoPIDCommand());
    NamedCommands.registerCommand(
        "HP Reload", new WaitCommand(IntakePreferences.outpostReloadWait.getValue()));
    NamedCommands.registerCommand(
        "Mowing",
        intake.collectNoPIDCommand()); // make a parameter that runs this action for a set amount of
    // time when at mid-field
    autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
    autoChooser.addOption("Auton Shoot", autonShootCommand);
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

    // Rumble driver controller when teleop starts
    RobotModeTriggers.teleop().onTrue(uiFeedback.timedRumbleCommand(driverJoystick.getHID(), 5.0));

    configureSubsystemDefaultCommands();
    configureTeleopBindings();
  }

  public void configureSubsystemDefaultCommands() {
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

  public void removeSubsystemDefaultCommands() {
    drivetrain.removeDefaultCommand();
  }

  public void configureTestBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // driverJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // driverJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // operatorJoystick.y().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // operatorJoystick.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // operatorJoystick.b().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // operatorJoystick.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    SmartDashboard.putData(shooter.startShooterTuningCommand());
    SmartDashboard.putData(shooter.stopShooterTuningCommand());
    SmartDashboard.putData(shooter.increaseFlywheelCommand());
    SmartDashboard.putData(shooter.decreaseFlywheelCommand());
    SmartDashboard.putData(shooter.increaseHoodCommand());
    SmartDashboard.putData(shooter.decreaseHoodCommand());
    SmartDashboard.putData(drivetrain.wheelRadiusCharacterization());

    driverJoystick
        .a()
        .whileTrue(driveAndLaunchCommand)
        .onFalse(shooter.stopFlywheelCommand().andThen(shooter.stowHood()));

    driverJoystick.b().onTrue(intake.testRollerNoPID()).onFalse(intake.stopRollerNoPID());

    driverJoystick.x().onTrue(intake.spinRollerCommand()).onFalse(intake.stopRollerCommand());
  }

  public void configureTeleopBindings() {

    driverJoystick
        .rightBumper()
        // .whileTrue(intake.setExtendNoPID())
        .onTrue(
            intake
                .setIntakeExtensionCommand(IntakeConstants.INTAKE_FORWARD_LIMIT)
                .andThen(intake.stopExtensionNoPID().andThen(intake.startRollerNoPID())));

    driverJoystick
        .leftBumper()
        .onTrue(
            intake
                .stopRollerNoPID()
                .andThen(intake.setIntakeExtensionCommand(IntakeConstants.INTAKE_REVERSE_LIMIT)));

    driverJoystick
        .b()
        .whileTrue(
            intake
                .outtakeRollerNoPID()
                .alongWith(
                    indexer
                        .startIndexerReverseNoPID())) /* .alongWith(shooter.spinFlywheelCommand))*/
        .onFalse(intake.stopRollerNoPID().andThen(indexer.stopIndexerNoPID()));

    operatorJoystick
        .rightTrigger()
        .whileTrue(drivetrain.setXCommand().andThen(indexer.pulsingIndexCommand()))
        .onFalse(indexer.stopFullIndexingNoPID());

    operatorJoystick
        .leftTrigger()
        .whileTrue(driveAndLaunchCommand)
        .onFalse(shooter.stopFlywheelCommand().andThen(shooter.stowHood()));

    operatorJoystick
        .leftBumper()
        .whileTrue(shooter.spinFlywheelRanged())
        .onFalse(shooter.stopFlywheelCommand().andThen(shooter.stowHood()));

    // Reset the field-centric heading on start button press.
    driverJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    operatorJoystick
        .a()
        .onTrue(
            Commands.runOnce(
                () ->
                    LaunchState.getInstance()
                        .setTargetPose3d(Constants.FieldConstants.getHubTarget())));

    operatorJoystick
        .x()
        .onTrue(
            Commands.runOnce(
                () ->
                    LaunchState.getInstance()
                        .setTargetPose3d(Constants.FieldConstants.getLeftPassTarget())));

    operatorJoystick
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    LaunchState.getInstance()
                        .setTargetPose3d(Constants.FieldConstants.getRightPassTarget())));

    operatorJoystick.y().whileTrue(uiFeedback.manualRumbleCommand(driverJoystick.getHID()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private SwerveRequest.FieldCentric getDriveAndLaunchRequest() {
    LaunchRequest launchRequest = launchState.getLaunchRequest();
    double rotationalRate =
        launchRequest.getTargetRobotAngularVelocity().in(RadiansPerSecond)
            + DrivePreferences.autoAim_kP.getValue()
                * launchRequest
                    .getTargetRobotAngle()
                    .minus(driveState.getCurrentDriveStats().Pose.getRotation())
                    .getRadians()
            + DrivePreferences.autoAim_kD.getValue()
                * (launchRequest.getTargetRobotAngularVelocity().in(RadiansPerSecond)
                    - driveState.getFieldVelocity().omegaRadiansPerSecond);
    return DriveConstants.DEFAULT_DRIVE_REQUEST
        .withVelocityX(
            -1
                * Math.copySign(Math.pow(driverJoystick.getLeftY(), 2), driverJoystick.getLeftY())
                * DrivePreferences.autoAimMaxSpeed
                    .getValue()) // Drive forward with negative Y (forward)
        .withVelocityY(
            -1
                * Math.copySign(Math.pow(driverJoystick.getLeftX(), 2), driverJoystick.getLeftX())
                * DrivePreferences.autoAimMaxSpeed.getValue()) // Drive left with negative X (left)
        .withRotationalRate(rotationalRate)
        .withDeadband(DriveConstants.MAX_DRIVE_SPEED * 0.1);
  }
}
