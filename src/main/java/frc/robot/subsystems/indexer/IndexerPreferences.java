package frc.robot.subsystems.indexer;

import frc.robot.preferences.DoublePreference;

public class IndexerPreferences {
    
    private IndexerPreferences(){}

    public static DoublePreference indexSpeed =
      new DoublePreference( "Indexer/Index Speed", 0.1); // in rotations per second
}
