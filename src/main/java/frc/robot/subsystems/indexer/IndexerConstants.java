package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

final class IndexerConstants {

  private IndexerConstants() {}

  // TODO Replace with real id
  protected static final int INDEXER_MOTOR_LEADER_ID = 3;
  protected static final int INDEXER_MOTOR_FOLLOWER_ID = 9;

  // TODO: Tune motor

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

  public static MotorOutputConfigs createLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static MotorOutputConfigs createFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }
}
