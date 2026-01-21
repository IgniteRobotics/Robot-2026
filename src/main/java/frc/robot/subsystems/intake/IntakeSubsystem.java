package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem {
    private final TalonFX m_intakeRollerMotor;
    private final TalonFX m_intakeExtensionMotor;

    public IntakeSubsystem(){
        m_intakeRollerMotor = new TalonFX(IntakeConstants.kIntakeRollerMotor);
        m_intakeExtensionMotor = new TalonFX(IntakeConstants.kIntakeExtensionMotor);
    }

}
