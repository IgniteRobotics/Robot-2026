package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private double currentPositionTarget;
    private PositionTorqueCurrentFOC positionControl;

    public IntakeSubsystem(){
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
        extensionMotor = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID);
        extensionMotor.getConfigurator().apply(IntakeConstants.createExtensionMotorSlot0Configs());
        extensionMotor.setPosition(0);
        currentPositionTarget = 0;
        positionControl = new PositionTorqueCurrentFOC(0);
    }

    @Override
    public void periodic(){
        extensionMotor.setControl(positionControl.withPosition(currentPositionTarget));
    }

    public Command spinRollerCommand(){
        return runOnce(() -> rollerMotor.set(IntakePreferences.rollerIntakeSpeed.getValue())).withName("Spin Intake Roller");
    }

    public Command stopRollerCommand(){
        return runOnce(() -> rollerMotor.stopMotor()).withName("Stop Intake Roller");
    }

    private boolean atExtensionSetpoint(){
        return Math.abs(extensionMotor.getPosition().getValueAsDouble() - currentPositionTarget) < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
    }

    public Command setIntakePositionCommand(double position){
        return runOnce(() -> currentPositionTarget = position).andThen(Commands.waitUntil(() -> atExtensionSetpoint()));
    }

    public Command collectCommand(){
        return setIntakePositionCommand(IntakePreferences.intakeCollectPosition.getValue()).andThen(spinRollerCommand()).withName("Activate Intake Collection");
    }

    public Command stowCommand(){
        return stopRollerCommand().andThen(setIntakePositionCommand(0)).withName("Stow Intake");
    }





}
