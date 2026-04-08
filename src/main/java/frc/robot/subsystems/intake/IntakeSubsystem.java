package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX rollerLeader;
  private final TalonFX rollerFollower;
  private final TalonFX extensionMotor;

  @Logged(name = "Extension Target", importance = Importance.CRITICAL)
  private Angle extensionTarget = Rotations.of(IntakeConstants.INTAKE_REVERSE_LIMIT); // Rotations

  private PositionVoltage extensionControl;

  @Logged(name = "Extension Is Compliant", importance = Importance.CRITICAL)
  private boolean isCompliantMode;

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
    rollerLeader = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    rollerFollower = new TalonFX(IntakeConstants.ROLLER_FOLLOWER_MOTOR_ID);
    extensionMotor = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);

    rollerLeader.getConfigurator().apply(IntakeConstants.createRollerLeaderMotorOutputConfigs());
    rollerFollower
        .getConfigurator()
        .apply(IntakeConstants.createRollerFollowerMotorOutputConfigs());
    rollerLeader.getConfigurator().apply(IntakeConstants.createRollerMotorCurrentLimitsConfigs());
    rollerFollower.getConfigurator().apply(IntakeConstants.createRollerMotorCurrentLimitsConfigs());
    rollerLeader.getConfigurator().apply(IntakeConstants.createRollerMotorRampConfigs());
    rollerFollower.getConfigurator().apply(IntakeConstants.createRollerMotorRampConfigs());

    rollerFollower.setControl(
        new Follower(rollerLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorSlot0Configs());
    extensionMotor
        .getConfigurator()
        .apply(IntakeConstants.createExtensionSoftwareLimitSwitchConfigs());
    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorOutputConfigs());
    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorSlot1Configs());
    extensionMotor.getConfigurator().apply(IntakeConstants.creatClosedLoopRampsConfigs());
    extensionMotor
        .getConfigurator()
        .apply(IntakeConstants.createExtenstionMotorCurrentLimitsConfigs());

    extensionMotor.setPosition(0);
    extensionTarget = Rotations.of(0);
    extensionControl = new PositionVoltage(0);
    isCompliantMode = false;
  }

  @Override
  public void periodic() {
    if (extensionMotor.getStatorCurrent().getValueAsDouble()
            > IntakePreferences.resistanceCurrentLimit.getValue()
        && isCompliantMode
        && RobotModeTriggers.teleop().getAsBoolean())
      CommandScheduler.getInstance()
          .schedule(stopRollerNoPID().andThen(setIntakeExtensionCommand(0)));

    extensionMotor.setControl(
        extensionControl
            .withSlot(isCompliantMode ? 1 : 0)
            .withPosition(extensionTarget.in(Rotations)));
  }

  private void setRollerVoltage(double magnitude) {
    rollerLeader.setVoltage(magnitude);
  }

  public Command startRollerNoPID() {
    return runOnce(() -> rollerLeader.set(IntakePreferences.rollerIntakePercent.getValue()))
        .withName("Set Roller Percent");
  }

  public Command testRollerNoPID() {
    return runOnce(() -> rollerLeader.set(IntakePreferences.testRollerIntakePercent.getValue()))
        .withName("Set Roller Percent");
  }

  public Command outtakeRollerNoPID() {
    return run(() -> rollerLeader.set(IntakePreferences.rollerOuttakePercent.getValue()))
        .withName("Set Roller Percent");
  }

  public Command startRollerReverseNoPID() {
    return run(() -> rollerLeader.set(IntakePreferences.rollerOuttakePercent.getValue()))
        .withName("Set Roller Reverse Percent");
  }

  public Command stopRollerNoPID() {
    return runOnce(() -> rollerLeader.set(0)).withName("Stop Roller No PID");
  }

  @Logged(name = "Extension Setpoint", importance = Importance.CRITICAL)
  public boolean atExtensionSetpoint() {
    return Math.abs(extensionMotor.getPosition().getValueAsDouble() - extensionTarget.in(Rotations))
            < IntakeConstants.ALLOWABLE_EXTENSION_ERROR
        && extensionMotor.getStatorCurrent().getValueAsDouble()
            < IntakeConstants.COMPLIANT_RESISTANCE_CURRENT_LIMIT.in(Amps);
  }

  public Command setIntakeExtensionCommand(double rotations) {
    return runOnce(() -> isCompliantMode = false)
        .andThen(runOnce(() -> extensionTarget = Rotations.of(rotations)))
        .andThen(Commands.waitUntil(() -> atExtensionSetpoint()))
        .finallyDo(() -> isCompliantMode = true);
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
    return setIntakeExtensionCommand(IntakeConstants.INTAKE_FORWARD_LIMIT)
        .andThen(startRollerNoPID()) // Set to  spinRollerCommand() after PID tuning
        .withName("Activate Intake Collection");
  }

  public Command collectNoPIDCommand() {
    return setExtendNoPID()
        .raceWith(new WaitCommand(IntakePreferences.noPIDWait.getValue()))
        .andThen(stopExtensionNoPID())
        .andThen(startRollerNoPID())
        .withName("Activate Intake Collection (NOPID)");
  }

  public Command stowCommand() {
    return setIntakeExtensionCommand(IntakeConstants.INTAKE_REVERSE_LIMIT)
        .andThen(stopRollerNoPID())
        .withName("Stow Intake");
  }

  public Command dislodgeCommand() {
    return setIntakeExtensionCommand(IntakeConstants.INTAKE_FORWARD_LIMIT)
        .andThen(new WaitUntilCommand(() -> this.atExtensionSetpoint()))
        .andThen(setIntakeExtensionCommand(IntakePreferences.dislodgePosition.getValue()))
        .andThen(new WaitUntilCommand(() -> this.atExtensionSetpoint()))
        .repeatedly()
        .withName("Agitate Intake");
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
