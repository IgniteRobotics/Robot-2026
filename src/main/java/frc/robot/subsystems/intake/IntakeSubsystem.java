package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX rollerLeader;
  private final TalonFX rollerFollower;
  private final TalonFX extensionMotor;

  @Logged(name = "Roller Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity rollerVelocityTarget; // RotationsPerSecond

  private VelocityVoltage rollerControl;

  @Logged(name = "Extension Target", importance = Importance.CRITICAL)
  private Angle extensionTarget = Rotations.of(IntakeConstants.INTAKE_REVERSE_LIMIT); // Rotations

  private PositionTorqueCurrentFOC extensionControl;

  private MotionMagicTorqueCurrentFOC mmExtenstionControl;

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
    rollerLeader.getConfigurator().apply(IntakeConstants.createRollerMotorSlot0Configs());
    rollerFollower.getConfigurator().apply(IntakeConstants.createRollerMotorSlot0Configs());
    rollerLeader.getConfigurator().apply(IntakeConstants.createRollerMotorCurrentLimitsConfigs());
    rollerFollower.getConfigurator().apply(IntakeConstants.createRollerMotorCurrentLimitsConfigs());

    rollerFollower.setControl(
        new Follower(rollerLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    rollerVelocityTarget = RotationsPerSecond.of(0);
    rollerControl = new VelocityVoltage(0);

    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorSlot0Configs());
    extensionMotor
        .getConfigurator()
        .apply(IntakeConstants.createExtensionSoftwareLimitSwitchConfigs());
    extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorOutputConfigs());
    extensionMotor.getConfigurator().apply(IntakeConstants.createExtenstionMotionMagicConfigs());
    extensionMotor
        .getConfigurator()
        .apply(IntakeConstants.createExtenstionMotorCurrentLimitsConfigs());

    extensionMotor.setPosition(0);
    extensionTarget = Rotations.of(0);
    extensionControl = new PositionTorqueCurrentFOC(0);
    mmExtenstionControl = new MotionMagicTorqueCurrentFOC(0);
  }

  @Override
  public void periodic() {

    // rollerMotor.setControl(rollerControl.withVelocity(rollerVelocityTarget.in(RotationsPerSecond)));

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
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setIntakeExtensionCommand(double rotations) {
    return runOnce(() -> extensionTarget = Rotations.of(rotations));
    // .andThen(Commands.waitUntil(() -> atExtensionSetpoint()));
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
    return setIntakeExtensionCommand(IntakePreferences.intakeCollectPosition.getValue())
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
        .andThen(stopRollerCommand())
        // .andThen(setIntakeExtensionCommand(Rotations.of(0)).repeatedly())
        .withName("Stow Intake");
  }

  public Command stowNoPIDCommand() {
    return setRetractNoPID()
        .raceWith(new WaitCommand(1))
        .andThen(stopExtensionNoPID())
        .andThen(stopRollerNoPID())
        .withName("Activate Intake Stow (NOPID)");
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
