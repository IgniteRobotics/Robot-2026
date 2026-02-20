package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DrivetrainSubsystem extends CommandSwerveDrivetrain {
  private DriveState driveState = DriveState.getInstance();

  private final CANrange drive_canrange;

  public DrivetrainSubsystem() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);

    drive_canrange = new CANrange(9, "rio");

    applySteerGains();
    applyDriveGains();
    configureAutoBuilder();
    configureCANrange();
  }

  @Override
  public void periodic() {
    super.periodic();
    for (VisionMeasurement estimate :
        driveState.grabVisionEstimateList(CameraConstants.photonCameraName_Front)) {
      addVisionMeasurement(
          estimate.getEstimatedPose().estimatedPose.toPose2d(),
          estimate.getTimestamp(),
          estimate.getTrust());
    }
    driveState.adjustCurrentDriveStats(this.getStateCopy());

    SmartDashboard.putNumber(
        "Drive Canrange Distance", drive_canrange.getDistance(true).getValueAsDouble());

    SmartDashboard.putNumber("kP Preference Current", DrivePreferences.translation_kP.getValue());
  }

  public Command sysIdSteer() {
    return m_sysIdRoutineSteer
        .quasistatic(Direction.kForward)
        .withTimeout(SteerMotorConfigs.QUASISTATIC_TIMEOUT)
        .andThen(
            m_sysIdRoutineSteer
                .quasistatic(Direction.kReverse)
                .withTimeout(SteerMotorConfigs.QUASISTATIC_TIMEOUT))
        .andThen(
            m_sysIdRoutineSteer
                .dynamic(Direction.kForward)
                .withTimeout(SteerMotorConfigs.DYNAMIC_TIMEOUT))
        .andThen(
            m_sysIdRoutineSteer
                .dynamic(Direction.kReverse)
                .withTimeout(SteerMotorConfigs.DYNAMIC_TIMEOUT));
  }

  public Command sysIdTranslation() {
    return m_sysIdRoutineTranslation
        .quasistatic(Direction.kForward)
        .withTimeout(DriveMotorConfigs.QUASISTATIC_TIMEOUT)
        .andThen(
            m_sysIdRoutineTranslation
                .quasistatic(Direction.kReverse)
                .withTimeout(DriveMotorConfigs.QUASISTATIC_TIMEOUT))
        .andThen(
            m_sysIdRoutineTranslation
                .dynamic(Direction.kForward)
                .withTimeout(DriveMotorConfigs.DYNAMIC_TIMEOUT))
        .andThen(
            m_sysIdRoutineTranslation
                .dynamic(Direction.kReverse)
                .withTimeout(DriveMotorConfigs.DYNAMIC_TIMEOUT));
  }

  private void applySteerGains() {
    this.getModule(0)
        .getSteerMotor()
        .getConfigurator()
        .apply(SteerMotorConfigs.createFrontLeftSteerMotorSlot0Configs());
    this.getModule(1)
        .getSteerMotor()
        .getConfigurator()
        .apply(SteerMotorConfigs.createFrontRightSteerMotorSlot0Configs());
    this.getModule(2)
        .getSteerMotor()
        .getConfigurator()
        .apply(SteerMotorConfigs.createRearLeftSteerMotorSlot0Configs());
    this.getModule(3)
        .getSteerMotor()
        .getConfigurator()
        .apply(SteerMotorConfigs.createRearRightSteerMotorSlot0Configs());
  }

  private void applyDriveGains() {
    this.getModule(0)
        .getDriveMotor()
        .getConfigurator()
        .apply(DriveMotorConfigs.createFrontLeftDriveMotorSlot0Configs());
    this.getModule(1)
        .getDriveMotor()
        .getConfigurator()
        .apply(DriveMotorConfigs.createFrontRightDriveMotorSlot0Configs());
    this.getModule(2)
        .getDriveMotor()
        .getConfigurator()
        .apply(DriveMotorConfigs.createRearLeftDriveMotorSlot0Configs());
    this.getModule(3)
        .getDriveMotor()
        .getConfigurator()
        .apply(DriveMotorConfigs.createRearRightDriveMotorSlot0Configs());
  }

  public Command driveForward() {
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.RobotCentric forwardStraight =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityY(0);

    return runOnce(() -> this.seedFieldCentric())
        .andThen(applyRequest(() -> point.withModuleDirection(new Rotation2d(0.0))))
        .withTimeout(1)
        .andThen(applyRequest(() -> forwardStraight.withVelocityX(0.5)))
        .withTimeout(2)
        .andThen(applyRequest(() -> brake))
        .withTimeout(1);
  }

  public void setDrivePower(double power) {
    for (int i = 0; i < 4; i++) {
      this.getModule(i).getDriveMotor().set(power);
    }
  }

  public void printDriveMotorCurrents() {
    System.out.println(
        "Front Left current: "
            + this.getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
    System.out.println(
        "Front Right current: "
            + this.getModule(1).getDriveMotor().getStatorCurrent().getValueAsDouble());
    System.out.println(
        "Rear Left current: "
            + this.getModule(2).getDriveMotor().getStatorCurrent().getValueAsDouble());
    System.out.println(
        "Rear Right current: "
            + this.getModule(3).getDriveMotor().getStatorCurrent().getValueAsDouble());
  }

  public PIDController getTranslationPIDController() {
    PIDController controller =
        new PIDController(
            DrivePreferences.translation_kP.getValue(),
            0,
            DrivePreferences.translation_kD.getValue());
    controller.setTolerance(DriveConstants.TRANSLATION_ALIGN_TOLERANCE);
    return controller;
  }

  public PIDController getRotationPIDController() {
    PIDController controller =
        new PIDController(
            DrivePreferences.rotation_kP.getValue(), 0, DrivePreferences.rotation_kD.getValue());
    controller.setTolerance(DriveConstants.ROTATION_ALIGN_TOLERANCE);
    controller.enableContinuousInput(-180, 180); // Degrees
    return controller;
  }

  public void configureCANrange() {
    ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
    drive_canrange.getConfigurator().refresh(proximityParamsConfigs);
  }

  private void configureAutoBuilder() {
    try {
      SmartDashboard.putNumber("What we think it is", DrivePreferences.translation_kP.getValue());
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  new SwerveRequest.ApplyRobotSpeeds()
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(
                  DrivePreferences.translation_kP.getValue(),
                  0,
                  DrivePreferences.translation_kD.getValue()),
              // PID constants for rotation
              new PIDConstants(
                  DrivePreferences.rotation_kP.getValue(),
                  0,
                  DrivePreferences.rotation_kD.getValue())),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  public Command followPath(PathPlannerPath path) {
    this.resetPose(path.getStartingHolonomicPose().get());
    return AutoBuilder.followPath(path);
  }
}
