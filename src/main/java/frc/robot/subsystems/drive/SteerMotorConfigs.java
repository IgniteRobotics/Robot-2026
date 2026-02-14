package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

// Steer Motor Feedback and Feedforward configs
final class SteerMotorConfigs {

  private SteerMotorConfigs() {}

  // SYS ID TIMEOUTs
  protected static final int QUASISTATIC_TIMEOUT = 5;
  protected static final int DYNAMIC_TIMEOUT = 3;

  // FRONT LEFT
  protected static final double FL_STEER_KP = 37.623;
  protected static final double FL_STEER_KD = 1.4197;
  protected static final double FL_STEER_KS = 0.12771;
  protected static final double FL_STEER_KV = 2.3119;
  protected static final double FL_STEER_KA = 0.035046;

  protected static Slot0Configs createFrontLeftSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FL_STEER_KP;
    slot.kD = FL_STEER_KD;
    slot.kS = FL_STEER_KS;
    slot.kV = FL_STEER_KV;
    slot.kA = FL_STEER_KA;
    return slot;
  }

  // FRONT RIGHT
  protected static final double FR_STEER_KP = 54.475;
  protected static final double FR_STEER_KD = 2.8084;
  protected static final double FR_STEER_KS = 0.18581;
  protected static final double FR_STEER_KV = 2.2948;
  protected static final double FR_STEER_KA = 0.039424;

  protected static Slot0Configs createFrontRightSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FR_STEER_KP;
    slot.kD = FR_STEER_KD;
    slot.kS = FR_STEER_KS;
    slot.kV = FR_STEER_KV;
    slot.kA = FR_STEER_KA;
    return slot;
  }

  // REAR LEFT
  protected static final double RL_STEER_KP = 34.211;
  protected static final double RL_STEER_KD = 1.2123;
  protected static final double RL_STEER_KS = 0.17069;
  protected static final double RL_STEER_KV = 2.2845;
  protected static final double RL_STEER_KA = 0.038551;

  protected static Slot0Configs createRearLeftSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RL_STEER_KP;
    slot.kD = RL_STEER_KD;
    slot.kS = RL_STEER_KS;
    slot.kV = RL_STEER_KV;
    slot.kA = RL_STEER_KA;
    return slot;
  }

  // REAR RIGHT
  protected static final double RR_STEER_KP = 30.411;
  protected static final double RR_STEER_KD = 0.93784;
  protected static final double RR_STEER_KS = 0.14977;
  protected static final double RR_STEER_KV = 2.2835;
  protected static final double RR_STEER_KA = 0.039078;

  protected static Slot0Configs createRearRightSteerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RR_STEER_KP;
    slot.kD = RR_STEER_KD;
    slot.kS = RR_STEER_KS;
    slot.kV = RR_STEER_KV;
    slot.kA = RR_STEER_KA;
    return slot;
  }
}
