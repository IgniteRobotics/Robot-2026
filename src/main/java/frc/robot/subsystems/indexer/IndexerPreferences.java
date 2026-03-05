package frc.robot.subsystems.indexer;

import frc.robot.preferences.DoublePreference;

final class IndexerPreferences {

  private IndexerPreferences() {}

  protected static DoublePreference indexSpeed =
      new DoublePreference("Indexer/Index Speed", 0.1); // in rotations per second

  protected static DoublePreference indexerPercent =
      new DoublePreference("Indexer/Index Percent (for without PID)", 0.1); // in percent

  protected static DoublePreference indexerReversePercent =
      new DoublePreference("Indexer/Reverse Index Percent (for without PID)", -0.1); // in percent

  protected static DoublePreference acceleratorPercent =
      new DoublePreference("Indexer/Accelerator Percent (for without PID)", 0.1); // in percent

  protected static DoublePreference indexerRunTime =
      new DoublePreference("Indexer/Pulsing Run Time", 2.0); // in seconds

  protected static DoublePreference indexerPauseTime =
      new DoublePreference("Indexer/Pulsing Pause Time", 0.1); // in seconds
}
