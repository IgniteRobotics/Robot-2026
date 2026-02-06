package frc.robot.subsystems.drive;

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
}
