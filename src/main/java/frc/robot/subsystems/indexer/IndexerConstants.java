package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.Slot0Configs;
final class IndexerConstants {

  private IndexerConstants(){}

  // TODO Replace with real id
  protected static final int INDEXER_MOTOR_ID = 6;

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
}
