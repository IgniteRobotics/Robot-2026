package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

final class ClimberConstants {

    private ClimberConstants() {}

    // TODO: Change to actual port
    protected static final int CLIMB_MOTOR_ID = 10;


    //TODO: Tune motor
    protected static final double ALLOWABLE_CLIMB_ERROR = 0.1;
    protected static final double CLIMB_FORWARD_LIMIT = 100;
    protected static final double CLIMB_REVERSE_LIMIT = 0;
    protected static final double CLIMB_KS = 0;
    protected static final double CLIMB_KP = 0;
    protected static final double CLIMB_KD = 0;

    protected static Slot0Configs createClimbMotorSlot0Configs() {
        Slot0Configs slot = new Slot0Configs();
        slot.kS = CLIMB_KS;
        slot.kP = CLIMB_KP;
        slot.kD = CLIMB_KD;
        return slot;
    }

    protected static SoftwareLimitSwitchConfigs createClimbSoftwareLimitSwitchConfigs() {
        SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
        configs.ForwardSoftLimitEnable = false;
        configs.ReverseSoftLimitEnable = false;
        configs.ForwardSoftLimitThreshold = CLIMB_FORWARD_LIMIT;
        configs.ReverseSoftLimitThreshold = CLIMB_REVERSE_LIMIT;
        return configs;
    }

    protected static final double SAFE_HOMING_EFFORT = -0.2;
    protected static final double SAFE_STATOR_LIMIT = 0.8;
}
