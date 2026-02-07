package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DrivetrainSubsystem extends CommandSwerveDrivetrain {
  private DriveState driveState = DriveState.getInstance();

  public DrivetrainSubsystem() {
    super(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
    applySteerGains();
    applyDriveGains();
  }

  @Override
  public void periodic() {
    super.periodic();
    for (VisionMeasurement estimate :
        driveState.grabVisionEstimateList(CameraConstants.photonCameraName1)) {
      addVisionMeasurement(
          estimate.getEstimatedPose().estimatedPose.toPose2d(),
          estimate.getTimestamp(),
          estimate.getTrust());
    }
    for (VisionMeasurement estimate :
        driveState.grabVisionEstimateList(CameraConstants.photonCameraName2)) {
      addVisionMeasurement(
          estimate.getEstimatedPose().estimatedPose.toPose2d(),
          estimate.getTimestamp(),
          estimate.getTrust());
    }
    driveState.adjustCurrentDriveStats(this.getStateCopy());
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

  private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    new SwerveRequest.ApplyFieldSpeeds().withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command followPath(PathPlannerPath path){
        this.resetPose(path.getStartingHolonomicPose().get());
        return AutoBuilder.followPath(path);
    }

}
