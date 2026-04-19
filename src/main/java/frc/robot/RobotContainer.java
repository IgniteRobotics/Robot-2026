// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.WheelSlipTest;
import frc.robot.statemachines.DriveState;
import frc.robot.statemachines.LaunchState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivePreferences;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakePreferences;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lemon_hunter.LemonHunterSubsystem;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.ui.UISubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  private final Telemetry logger = new Telemetry(DriveConstants.MAX_DRIVE_SPEED);

  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final CommandXboxController operatorJoystick = new CommandXboxController(1);

  @Logged(name = "Drivetrain")
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

  @Logged(name = "Intake")
  public final IntakeSubsystem intake = new IntakeSubsystem();

  @Logged(name = "Indexer")
  public final IndexerSubsystem indexer = new IndexerSubsystem();

  @Logged(name = "Shooter")
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  @Logged(name = "Vision")
  public final VisionSubsystem vision = new VisionSubsystem();

  @Logged(name = "Hunter")
  public final LemonHunterSubsystem hunter = new LemonHunterSubsystem();

  @Logged(name = "UI Feedback")
  public final UISubsystem uiFeedback =
      new UISubsystem(driverJoystick.getHID(), operatorJoystick.getHID());

  /*
  @Logged(name = "PDH")
  public final PDHSubsystem pdh = new PDHSubsystem();*/

  private final DriveState driveState = DriveState.getInstance();
  private final LaunchState launchState = LaunchState.getInstance();

  private final Command driveAndLaunchCommand =
      drivetrain
          .applyRequest(() -> getDriveAndLaunchRequest())
          // .alongWith(shooter.spinFlywheelCommand());
          .alongWith(shooter.spinFlywheelRanged())
          .withName("Drive and Launch");

  private final Command autonShootCommand =
      drivetrain
          .applyRequest(() -> getDriveAndLaunchRequest())
          // .alongWith(shooter.spinFlywheelCommand());
          .alongWith(shooter.spinFlywheelRanged())
          .alongWith(new WaitCommand(0.5).andThen(indexer.startFullIndexingNoPID()));

  private final Command stopShotCommand =
      indexer
          .stopFullIndexingNoPID()
          .andThen(shooter.stopFlywheelCommand())
          .andThen(shooter.stowHood());

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Seed", drivetrain.runOnce(drivetrain::seedFieldCentric));
    NamedCommands.registerCommand("AutonShoot", autonShootCommand);
    NamedCommands.registerCommand("StopShot", stopShotCommand);
    NamedCommands.registerCommand("StopRoller", intake.stopRollerNoPID());
    NamedCommands.registerCommand("Collect Intake", intake.collectCommand());
    NamedCommands.registerCommand(
        "Stow Intake", intake.stopRollerNoPID().andThen(intake.stowCommand()));
    NamedCommands.registerCommand(
        "HP Reload", new WaitCommand(IntakePreferences.outpostReloadWait.getValue()));
    autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

    // Rumble driver controller when teleop starts
    // Add a trigger to auto set the target to the hub on teleop start.
    RobotModeTriggers.teleop()
        .onTrue(
            uiFeedback
                .timedRumbleCommand(driverJoystick.getHID(), 5.0)
                .alongWith(
                    Commands.runOnce(
                        () ->
                            LaunchState.getInstance()
                                .setTargetPose3d(Constants.FieldConstants.getHubTarget())))
                .withName("Rumble & Set Pose"));

    // Adds SmartDashboard Testing/Tuning Commands
    addSubsystemTests();
    addShooterTuningCommands();
    addSysIdRoutines();

    // Binds controllers to commands
    configureDefaultDrivetrainCommand();
    configureBindings();

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureDefaultDrivetrainCommand() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    DriveConstants.DEFAULT_DRIVE_REQUEST
                        .withVelocityX(
                            -1
                                * Math.copySign(
                                    Math.pow(driverJoystick.getLeftY(), 2),
                                    driverJoystick.getLeftY())
                                * DriveConstants
                                    .MAX_DRIVE_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -1
                                * Math.copySign(
                                    Math.pow(driverJoystick.getLeftX(), 2),
                                    driverJoystick.getLeftX())
                                * DriveConstants
                                    .MAX_DRIVE_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(
                            -1
                                * Math.copySign(
                                    Math.pow(driverJoystick.getRightX(), 2),
                                    driverJoystick.getRightX())
                                * DriveConstants
                                    .MAX_ANGULAR_SPEED) // Drive counterclockwise with negative X
                        // (left)
                        .withDeadband(DriveConstants.MAX_DRIVE_SPEED * 0.1)
                        .withRotationalDeadband(DriveConstants.MAX_ANGULAR_SPEED * 0.1))
            .withName("Teleop Drive"));
  }

  public void addSubsystemTests() {
    SmartDashboard.putData(shooter.testCommand());
    SmartDashboard.putData(indexer.testCommand());
    SmartDashboard.putData(intake.testCommand());
    SmartDashboard.putData(drivetrain.wheelRadiusCharacterization());
    SmartDashboard.putData(new WheelSlipTest(drivetrain));
  }

  public void addShooterTuningCommands() {
    SmartDashboard.putData(shooter.startShooterTuningCommand());
    SmartDashboard.putData(shooter.stopShooterTuningCommand());
    SmartDashboard.putData(shooter.increaseFlywheelCommand());
    SmartDashboard.putData(shooter.decreaseFlywheelCommand());
    SmartDashboard.putData(shooter.increaseHoodCommand());
    SmartDashboard.putData(shooter.decreaseHoodCommand());
  }

  public void addSysIdRoutines() {
    SmartDashboard.putData(shooter.sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(shooter.sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData(shooter.sysIdDynamic(Direction.kForward));
    SmartDashboard.putData(shooter.sysIdDynamic(Direction.kReverse));

    SmartDashboard.putData("Start Logger", Commands.runOnce(SignalLogger::start));
    SmartDashboard.putData("Stop Logger", Commands.runOnce(SignalLogger::stop));
  }

  public void configureBindings() {
    driverJoystick
        .rightBumper()
        // .whileTrue(intake.setExtendNoPID())
        .onTrue(intake.collectCommand())
        .onFalse(intake.stopRollerNoPID());

    driverJoystick
        .leftBumper()
        .onTrue(intake.stopRollerNoPID().andThen(intake.stowCommand()).withName("Stow Intake"));

    driverJoystick.a().onTrue(intake.agitateCommand());

    // outtake fuel.  don't retract when done.
    driverJoystick
        .b()
        .onTrue(
            intake
                .extendCommand()
                .andThen(
                    intake.startRollerReverseNoPID().alongWith(indexer.startIndexerReverseNoPID()))
                .withName("Outtake"))
        .onFalse(
            intake.stopRollerNoPID().andThen(indexer.stopIndexerNoPID()).withName("Stop Roller"));

    DoubleSupplier spinVx =
        () ->
            -Math.copySign(Math.pow(driverJoystick.getLeftY(), 2), driverJoystick.getLeftY())
                * DriveConstants.MAX_DRIVE_SPEED;
    DoubleSupplier spinVy =
        () ->
            -Math.copySign(Math.pow(driverJoystick.getLeftX(), 2), driverJoystick.getLeftX())
                * DriveConstants.MAX_DRIVE_SPEED;

    driverJoystick
        .leftTrigger()
        .whileTrue(drivetrain.spinMoveCommand(true, spinVx, spinVy).withTimeout(3.0));
    driverJoystick
        .rightTrigger()
        .whileTrue(drivetrain.spinMoveCommand(false, spinVx, spinVy).withTimeout(3.0));

    operatorJoystick
        .rightTrigger()
        .whileTrue(indexer.startFullIndexingNoPID().withName("Lock Wheels and Index"));

    operatorJoystick
        .leftTrigger()
        .whileTrue(driveAndLaunchCommand)
        .onFalse(shooter.stopFlywheelCommand().andThen(shooter.stowHood()));

    operatorJoystick.leftBumper().whileTrue(drivetrain.setXCommand());

    operatorJoystick
        .rightBumper()
        .whileTrue(shooter.spinFlywheelPostCommand())
        .onFalse(shooter.stopFlywheelCommand().andThen(shooter.stowHood()));

    // Reset the field-centric heading on start button press.
    driverJoystick
        .start()
        .onTrue(
            drivetrain
                .runOnce(drivetrain::seedFieldCentric)
                .alongWith(vision.resetPose())
                .withName("Seed & Reset Pose"))
        .onFalse(vision.stopResetPose());

    operatorJoystick
        .a()
        .onTrue(
            Commands.runOnce(
                    () ->
                        LaunchState.getInstance()
                            .setTargetPose3d(Constants.FieldConstants.getHubTarget()))
                .withName("Set Target To Hub"));

    operatorJoystick
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        LaunchState.getInstance()
                            .setTargetPose3d(Constants.FieldConstants.getLeftPassTarget()))
                .withName("Set Target To Left Pass"));

    operatorJoystick
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        LaunchState.getInstance()
                            .setTargetPose3d(Constants.FieldConstants.getRightPassTarget()))
                .withName("Set Target To Right Pass"));

    operatorJoystick
        .y()
        .whileTrue(
            uiFeedback
                .manualRumbleCommand(driverJoystick.getHID())
                .withName("Rumble Driver Controller"));
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

  private SwerveRequest.FieldCentric getDriveToLemonRequest() {
    Pose2d clusterPose = hunter.getClusterCentroid(driveState.getCurrentDriveStats().Pose);
    Translation2d translation = driveState.getCurrentDriveStats().Pose.minus(clusterPose).getTranslation();
    AngularVelocity targetAngularVelocity = RadiansPerSecond.of(
            translation.getAngle()
                .minus(driveState.getPreviousDriveStats().Pose.getRotation())
                .getRadians());
   
    double rotationalRate =
        targetAngularVelocity.in(RadiansPerSecond)
        + DrivePreferences.autoAim_kP.getValue()
            * translation.getAngle()
                .minus(driveState.getCurrentDriveStats().Pose.getRotation())
                    .getRadians()
        + DrivePreferences.autoAim_kD.getValue()
            * (targetAngularVelocity.in(RadiansPerSecond)
                - driveState.getFieldVelocity().omegaRadiansPerSecond);


    return DriveConstants.DEFAULT_DRIVE_REQUEST
        .withVelocityX(-1 * DriveConstants.HUNT_SPEED) // Drive forward with negative Y (forward)
        .withVelocityY(0) // Lock horizontal driving
        .withRotationalRate(rotationalRate)
        .withDeadband(DriveConstants.MAX_DRIVE_SPEED * 0.1);
  }
}
