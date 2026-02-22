package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeConstants;

@Logged
public class SimpleShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotorLeader;
  private final TalonFX flywheelMotorFollower;
  private final TalonFX hoodMotor;

  @Logged(name = "Hood Target", importance = Importance.CRITICAL)
  private Angle hoodTarget; // Rotations

  private PositionTorqueCurrentFOC hoodControl;

  final SysIdRoutine m_sysIdRoutineFlywheel =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setFlywheelVoltage(output.magnitude()), null, this));

  public SimpleShooterSubsystem() {
    flywheelMotorLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID);
    flywheelMotorFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createLeaderMotorOutputConfigs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFollowerMotorOutputConfigs());

    flywheelMotorFollower.setControl(
        new Follower(flywheelMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorOutputConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
    hoodTarget = Rotations.of(0);
    hoodControl = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public void periodic() {
    hoodMotor.setControl(hoodControl.withPosition(hoodTarget.in(Rotations)));
  }

  // for sysID
  private void setFlywheelVoltage(double magnitude) {
    flywheelMotorLeader.setVoltage(magnitude);
    flywheelMotorFollower.setVoltage(magnitude);
  }

  @Logged(name = "At Hood Setpoint", importance = Importance.CRITICAL)
  public boolean atHoodSetpoint() {
    return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodTarget.in(Rotations))
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setHoodCommand(Angle position) {
    return runOnce(() -> hoodTarget = position).withName("Set Hood Angle");
  }

  public Command launchLemonsCommandNoPID() {
    return setHoodCommand(Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue()))
        .alongWith(
            run(
                () -> {
                  flywheelMotorLeader.set(ShooterPreferences.flywheelLaunchPercent.getValue());
                  flywheelMotorFollower.set(ShooterPreferences.flywheelLaunchPercent.getValue());
                }))
        .withName("Start Launching Lemons (No PID)");
  }

  public Command stopLaunchLemonsNoPIDCommand() {
    return setHoodCommand(Rotations.of(0))
        .alongWith(
            run(
                () -> {
                  flywheelMotorLeader.set(0);
                  flywheelMotorFollower.set(0);
                }))
        .withName("Stop Launching Lemons (No PID)");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.dynamic(direction);
  }
}
