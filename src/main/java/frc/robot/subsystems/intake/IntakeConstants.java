package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;

public class IntakeConstants {
    
    private IntakeConstants(){}

    //TODO: Change to actual ports
    public static final int ROLLER_MOTOR_ID = 2; 
    public static final int EXTENSION_MOTOR_ID = 3;

    //TODO: Tune
    public static final double ALLOWABLE_EXTENSION_ERROR = 0.1;
    public static final double EXTENSION_kP = 0;
    public static final double EXTENSION_kD = 0;
    public static Slot0Configs createExtensionMotorSlot0Configs(){
        Slot0Configs slot = new Slot0Configs();
        slot.kP = EXTENSION_kP;
        slot.kD = EXTENSION_kD;
        return slot;
    }


}
