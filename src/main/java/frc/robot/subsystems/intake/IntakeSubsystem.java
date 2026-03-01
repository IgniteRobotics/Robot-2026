package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final TalonFX extensionMotor;

  @Logged(name = "Roller Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity rollerVelocityTarget; // RotationsPerSecond

  private VelocityVoltage rollerControl;

  @Logged(name = "Extension Target", importance = Importance.CRITICAL)
  private Angle extensionTarget; // Rotations

  private PositionTorqueCurrentFOC extensionControl;

  final SysIdRoutine m_sysIdRoutineRoller =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRoller_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setRollerVoltage(output.magnitude()), null, this));

  public IntakeSubsystem() {
    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    extensionMotor = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);

    rollerMotor.getConfigurator().apply(IntakeConstants.createRollerMotorSlot0Configs());
    rollerVelocityTarget = RotationsPerSecond.of(0);
    rollerControl = new VelocityVoltage(0);

    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorSlot0Configs());
    extensionMotor
        .getConfigurator()
        .apply(IntakeConstants.createExtensionSoftwareLimitSwitchConfigs());
    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorOutputConfigs());
    extensionMotor.setPosition(0);
    extensionTarget = Rotations.of(0);
    extensionControl = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public void periodic() {
    rollerMotor.setControl(rollerControl.withVelocity(rollerVelocityTarget.in(RotationsPerSecond)));
    extensionMotor.setControl(
        extensionControl
            .withPosition(extensionTarget.in(Rotations))
            .withOverrideCoastDurNeutral(true));
  }

  public Command spinRollerCommand() {
    return runOnce(
            () ->
                rollerVelocityTarget =
                    RotationsPerSecond.of(IntakePreferences.rollerIntakeSpeed.getValue()))
        .withName("Spin Intake Roller");
  }

  public Command stopRollerCommand() {
    return runOnce(() -> rollerVelocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Intake Roller");
  }

  private void setRollerVoltage(double magnitude) {
    rollerMotor.setVoltage(magnitude);
  }

  public Command startRollerNoPID() {
    return run(() -> rollerMotor.set(IntakePreferences.rollerIntakePercent.getValue()))
        .withName("Set Roller Percent");
  }

  public Command startRollerReverseNoPID(){ 
    return run(() -> rollerMotor.set(IntakePreferences.rollerOutakePercent.getValue()))
        .withName("Set Roller Reverse Percent");
  }

  public Command stopRollerNoPID() {
    return run(() -> rollerMotor.set(0)).withName("Stop Roller No PID");
  }

  @Logged(name = "Extension Setpoint", importance = Importance.CRITICAL)
  public boolean atExtensionSetpoint() {
    return Math.abs(extensionMotor.getPosition().getValueAsDouble() - extensionTarget.in(Rotations))
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setIntakeExtensionCommand(Angle position) {
    return runOnce(() -> extensionTarget = position)
        .andThen(Commands.waitUntil(() -> atExtensionSetpoint()));
  }

  public Command setExtendNoPID() {
    return run(() -> extensionMotor.set(IntakePreferences.extendPercent.getValue()))
        .withName("Extend Intake Percent");
  }

  public Command setRetractNoPID() {
    return run(() -> extensionMotor.set(IntakePreferences.retractPercent.getValue()))
        .withName("Retract Intake Percent");
  }

  public Command stopExtensionNoPID() {
    return runOnce(() -> extensionMotor.set(0)).withName("Stop Intake Percent");
  }

  public Command collectCommand() {
    return setIntakeExtensionCommand(
            Rotations.of(IntakePreferences.intakeCollectPosition.getValue()))
        .andThen(spinRollerCommand())
        .withName("Activate Intake Collection");
  }

  public Command stowCommand() {
    return setIntakeExtensionCommand(Rotations.of(0))
        .andThen(stopRollerCommand())
        .andThen(setIntakeExtensionCommand(Rotations.of(0)).repeatedly())
        .withName("Stow Intake");
  }

  public Command dislodgeCommand() {
    return spinRollerCommand()
        .andThen(
            setIntakeExtensionCommand(Rotations.of(IntakePreferences.dislodgePosition.getValue()))
                .andThen(
                    setIntakeExtensionCommand(
                        Rotations.of(IntakePreferences.intakeCollectPosition.getValue()))))
        .repeatedly()
        .withName("Dislodge Intake");
  }

  public Command homeIntakeCommand() {
    return runEnd(
            () -> extensionMotor.set(IntakeConstants.SAFE_HOMING_EFFORT),
            () -> extensionMotor.setPosition(0))
        .until(
            () -> {
              return extensionMotor.getStatorCurrent().getValueAsDouble()
                  > IntakeConstants.SAFE_STATOR_LIMIT;
            });
  }
}
