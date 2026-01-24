package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;

public class ShooterConstants {
    
  // TODO: Change to actual ports
  public static final int FLYWHEEL_MOTOR_ID = 4;
  public static final int HOOD_MOTOR_ID = 5;

  // TODO: Tune Flywheel and Hood Motor

  // Flywheel motor
  public static final double FLYWHEEL_KS = 0;
  public static final double FLYWHEEL_KV = 0;
  public static final double FLYWHEEL_KP = 0;
  public static final double FLYWHEEL_KD = 0;

  public static Slot0Configs createRollerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = FLYWHEEL_KS;
    slot.kV = FLYWHEEL_KV;
    slot.kP = FLYWHEEL_KP;
    slot.kD = FLYWHEEL_KD;
    return slot;
  }

  
}
