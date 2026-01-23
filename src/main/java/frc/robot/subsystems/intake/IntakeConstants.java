package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;

public class IntakeConstants {

  private IntakeConstants() {}

  // TODO: Change to actual ports
  public static final int ROLLER_MOTOR_ID = 2;
  public static final int EXTENSION_MOTOR_ID = 3;

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
}
