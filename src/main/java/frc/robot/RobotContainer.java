// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveAndLaunch;
import frc.robot.statemachines.LaunchState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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

  private final LaunchState launchState = LaunchState.getInstance();

  private final DriveAndLaunch driveAndLaunchCommand =
      new DriveAndLaunch(
          drivetrain,
          () -> driverJoystick.getLeftY(),
          () -> driverJoystick.getLeftY(),
          () -> launchState.getTargetPose3d());

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Seed", drivetrain.runOnce(drivetrain::seedFieldCentric));
    autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
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

    driverJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::start));
    driverJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    operatorJoystick.y().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    operatorJoystick.a().whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    operatorJoystick.b().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    operatorJoystick.x().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }

  public void configureTeleopBindings() {

    driverJoystick
        .rightBumper()
        .whileTrue(intake.setExtendNoPID())
        .onFalse(intake.stopExtensionNoPID().andThen(intake.startRollerNoPID()));

    driverJoystick
        .leftBumper()
        .whileTrue(intake.setRetractNoPID())
        .onFalse(intake.stopExtensionNoPID().andThen(intake.stopRollerNoPID()));

    // driverJoystick
    //     .leftTrigger()
    //     .whileTrue(shooter.launchLemonsCommand())
    //     .onFalse(shooter.stopLaunchLemonsNoPIDCommand());

    driverJoystick.leftTrigger().whileTrue(driveAndLaunchCommand);

    driverJoystick
        .rightTrigger()
        .whileTrue(indexer.startFullIndexingNoPID())
        .onFalse(indexer.stopFullIndexingNoPID());

    // operatorJoystick.leftTrigger().whileTrue(driveAndLaunchCommand.repeatedly());
    // operatorJoystick
    //     .rightBumper()
    //     .whileTrue(indexer.startFullIndexingNoPID())
    //     .onFalse(indexer.stopFullIndexingNoPID());

    // Reset the field-centric heading on start button press.
    driverJoystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
