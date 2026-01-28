package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

public class ShooterConstants {

  private ShooterConstants() {}

  // TODO: Change to actual ports
  public static final int FLYWHEEL_MOTOR_ID = 4;
  public static final int HOOD_MOTOR_ID = 5;

  // TODO: Tune Flywheel and Hood Motor

  // Flywheel motor
  public static final double FLYWHEEL_KS = 0;
  public static final double FLYWHEEL_KV = 0;
  public static final double FLYWHEEL_KP = 0;
  public static final double FLYWHEEL_KD = 0;

  public static Slot0Configs createFlywheelMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = FLYWHEEL_KS;
    slot.kV = FLYWHEEL_KV;
    slot.kP = FLYWHEEL_KP;
    slot.kD = FLYWHEEL_KD;
    return slot;
  }

  public static final double ALLOWABLE_HOOD_ERROR = 0.1;
  public static final double HOOD_FORWARD_LIMIT = 100;
  public static final double HOOD_REVERSE_LIMIT = 0;
  public static final double HOOD_KS = 0;
  public static final double HOOD_KP = 0;
  public static final double HOOD_KD = 0;

  public static Slot0Configs createHoodMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = HOOD_KS;
    slot.kP = HOOD_KP;
    slot.kD = HOOD_KD;
    return slot;
  }

  public static SoftwareLimitSwitchConfigs createHoodSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = false;
    configs.ReverseSoftLimitEnable = false;
    configs.ForwardSoftLimitThreshold = HOOD_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = HOOD_REVERSE_LIMIT;
    return configs;
  }

  public static final double SAFE_HOMING_EFFORT = -0.2; // Update these
  public static final double SAFE_STATOR_LIMIT = 0.8; // Update these
}
