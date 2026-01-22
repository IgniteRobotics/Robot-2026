package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem {
    private final TalonFX m_intakeRollerMotor;
    private final TalonFX m_intakeExtensionMotor;

    public IntakeSubsystem(){
        m_intakeRollerMotor = new TalonFX(IntakeConstants.kIntakeRollerMotor);
        m_intakeExtensionMotor = new TalonFX(IntakeConstants.kIntakeExtensionMotor);
    }

    public void spinRoller(){
        m_intakeRollerMotor.set(IntakePreferences.rollerIntakeSpeed.getValue());
    }

    public void stopRoller(){
        m_intakeRollerMotor.stopMotor();
    }

    public void setIntakePosition(double position){
        m_intakeExtensionMotor.setPosition(position);
    }

    public void collect(){
        spinRoller();
        setIntakePosition(IntakePreferences.intakeCollectPosition.getValue());
    }

    public void stow(){
        stopRoller();
        setIntakePosition(0);
    }





}
