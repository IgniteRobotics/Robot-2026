package frc.robot.subsystems.indexer;

import frc.robot.preferences.DoublePreference;

final class IndexerPreferences {

  private IndexerPreferences() {}

  protected static DoublePreference indexSpeed =
      new DoublePreference("Indexer/Index Speed", 0.1); // in rotations per second

  public static DoublePreference indexerPercent =
      new DoublePreference("Indexer/Index Percent (for without PID)", 0.1); // in percent
}
