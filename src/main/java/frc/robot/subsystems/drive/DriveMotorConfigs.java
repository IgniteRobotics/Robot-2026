package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;

final class DriveMotorConfigs {
  private DriveMotorConfigs() {}

  // SYS ID TIMEOUTs
  protected static final int QUASISTATIC_TIMEOUT = 5;
  protected static final int DYNAMIC_TIMEOUT = 3;

  // FRONT LEFT
  protected static final double FL_DRIVE_KP = 0.13679;
  protected static final double FL_DRIVE_KD = 0;
  protected static final double FL_DRIVE_KS = 0.064073;
  protected static final double FL_DRIVE_KV = 0.11593;
  protected static final double FL_DRIVE_KA = 0.0075066;

  protected static Slot0Configs createFrontLeftDriveMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FL_DRIVE_KP;
    slot.kD = FL_DRIVE_KD;
    slot.kS = FL_DRIVE_KS;
    slot.kV = FL_DRIVE_KV;
    slot.kA = FL_DRIVE_KA;
    return slot;
  }

  // FRONT RIGHT
  protected static final double FR_DRIVE_KP = 0.11688;
  protected static final double FR_DRIVE_KD = 0;
  protected static final double FR_DRIVE_KS = 0.074625;
  protected static final double FR_DRIVE_KV = 0.11585;
  protected static final double FR_DRIVE_KA = 0.004729;

  protected static Slot0Configs createFrontRightDriveMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = FR_DRIVE_KP;
    slot.kD = FR_DRIVE_KD;
    slot.kS = FR_DRIVE_KS;
    slot.kV = FR_DRIVE_KV;
    slot.kA = FR_DRIVE_KA;
    return slot;
  }

  // REAR LEFT
  protected static final double RL_DRIVE_KP = 0.11906;
  protected static final double RL_DRIVE_KD = 0;
  protected static final double RL_DRIVE_KS = 0.087043;
  protected static final double RL_DRIVE_KV = 0.11469;
  protected static final double RL_DRIVE_KA = 0.0046479;

  protected static Slot0Configs createRearLeftDriveMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RL_DRIVE_KP;
    slot.kD = RL_DRIVE_KD;
    slot.kS = RL_DRIVE_KS;
    slot.kV = RL_DRIVE_KV;
    slot.kA = RL_DRIVE_KA;
    return slot;
  }

  // REAR RIGHT
  protected static final double RR_DRIVE_KP = 0.12745;
  protected static final double RR_DRIVE_KD = 0;
  protected static final double RR_DRIVE_KS = 0.083162;
  protected static final double RR_DRIVE_KV = 0.11652;
  protected static final double RR_DRIVE_KA = 0.0056185;

  protected static Slot0Configs createRearRightDriveMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kP = RR_DRIVE_KP;
    slot.kD = RR_DRIVE_KD;
    slot.kS = RR_DRIVE_KS;
    slot.kV = RR_DRIVE_KV;
    slot.kA = RR_DRIVE_KA;
    return slot;
  }
}
