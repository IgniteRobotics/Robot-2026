package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

final class IndexerConstants {

  private IndexerConstants() {}

  protected static final int INDEXER_MOTOR_LEADER_ID = 4;
  protected static final int ACCELERATOR_MOTOR_ID = 3;

  // TODO: PID tune motor

  protected static final double INDEXER_KS = 0;
  protected static final double INDEXER_KV = 0;
  protected static final double INDEXER_KP = 0;
  protected static final double INDEXER_KD = 0;

  protected static Slot0Configs createIndexerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = INDEXER_KS;
    slot.kV = INDEXER_KV;
    slot.kP = INDEXER_KP;
    slot.kD = INDEXER_KD;
    return slot;
  }

  protected static final double ACCELERATOR_KS = 0;
  protected static final double ACCELERATOR_KV = 0;
  protected static final double ACCELERATOR_KP = 0;
  protected static final double ACCELERATOR_KD = 0;

  protected static Slot0Configs createAcceleratorMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = ACCELERATOR_KS;
    slot.kV = ACCELERATOR_KV;
    slot.kP = ACCELERATOR_KP;
    slot.kD = ACCELERATOR_KD;
    return slot;
  }

  public static MotorOutputConfigs createIndexerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final double INDEXER_STATOR_CURRENT_LIMIT = 60;
  public static final double INDEXER_SUPPLY_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createIndexerCurrentLimitsConfigs() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.StatorCurrentLimit = INDEXER_STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimit = INDEXER_SUPPLY_CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = true;
    configs.SupplyCurrentLimitEnable = true;
    return configs;
  }

  public static MotorOutputConfigs createAcceleratorMotorOutputsConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final double ACCELERATOR_STATOR_CURRENT_LIMIT = 50;
  public static final double ACCELERATOR_SUPPLY_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createAcceleratorCurrentLimitsConfigs() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.StatorCurrentLimit = ACCELERATOR_STATOR_CURRENT_LIMIT;
    configs.SupplyCurrentLimit = ACCELERATOR_SUPPLY_CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = true;
    configs.SupplyCurrentLimitEnable = true;
    return configs;
  }

  public static final double ACCELERATOR_POWER = 1;
  public static final double INDEXER_POWER = 1;
}
