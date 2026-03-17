package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

  private IntakeConstants() {}

  // TODO: Change to actual ports
  public static final int ROLLER_MOTOR_ID = 2;
  public static final int EXTENSION_MOTOR_ID = 1;

  // TODO: Tune Roller and Extension Motors

  // Roller Motor
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0;
  public static final double ROLLER_KP = 0;
  public static final double ROLLER_KD = 0;

  public static Slot0Configs createRollerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = ROLLER_KS;
    slot.kV = ROLLER_KV;
    slot.kP = ROLLER_KP;
    slot.kD = ROLLER_KD;
    return slot;
  }

  // Extension Motor
  public static final double ALLOWABLE_EXTENSION_ERROR = 0.1;
  public static final double INTAKE_FORWARD_LIMIT = 100;
  public static final double INTAKE_REVERSE_LIMIT = 0;
  public static final double EXTENSION_KS = 0;
  public static final double EXTENSION_KP = 0;
  public static final double EXTENSION_KD = 0;

  public static Slot0Configs createExtensionMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = EXTENSION_KS;
    slot.kP = EXTENSION_KP;
    slot.kD = EXTENSION_KD;
    return slot;
  }

  public static SoftwareLimitSwitchConfigs createExtensionSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = false;
    configs.ReverseSoftLimitEnable = false;
    configs.ForwardSoftLimitThreshold = INTAKE_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = INTAKE_REVERSE_LIMIT;
    return configs;
  }

  public static MotorOutputConfigs createExtensionMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    // newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static final double SAFE_HOMING_EFFORT = -0.2;
  public static final double SAFE_STATOR_LIMIT = 0.8;

  //PID shifting 
  drop1 = TalonFX.getConfiguration().apply(Slot0Configs, 0.005); // the double represent the total timeout the motor should have(curently set to 5ms) https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/configuration-guide.html
  drop2 = TalonFR.getConfiguration().apply(Slot0Configs, false);// from another sourse using a boolean false gets rid of the timeout

}
