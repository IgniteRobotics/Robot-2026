package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final TalonFX extensionMotor;

  private AngularVelocity rollerVelocityTarget; // RotationsPerSecond
  private VelocityVoltage rollerControl;

  private Angle extensionTarget; // Rotations
  private PositionTorqueCurrentFOC extensionControl;

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
    extensionMotor.setPosition(0);
    extensionTarget = Rotations.of(0);
    extensionControl = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public void periodic() {
    rollerMotor.setControl(rollerControl.withVelocity(rollerVelocityTarget.in(RotationsPerSecond)));
    extensionMotor.setControl(extensionControl.withPosition(extensionTarget.in(Rotations)));
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

  private boolean atExtensionSetpoint() {
    return Math.abs(extensionMotor.getPosition().getValueAsDouble() - extensionTarget.in(Rotations))
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setIntakeExtensionCommand(Angle position) {
    return runOnce(() -> extensionTarget = position)
        .andThen(Commands.waitUntil(() -> atExtensionSetpoint()));
  }

  public Command collectCommand() {
    return setIntakeExtensionCommand(
            Rotations.of(IntakePreferences.intakeCollectPosition.getValue()))
        .andThen(spinRollerCommand())
        .withName("Activate Intake Collection");
  }

  public Command stowCommand() {
    return stopRollerCommand()
        .andThen(setIntakeExtensionCommand(Rotations.of(0)))
        .withName("Stow Intake");
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
