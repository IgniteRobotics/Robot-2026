package frc.robot.subsystems.ui;

import frc.robot.preferences.DoublePreference;

public final class UIPreferences {

  private UIPreferences() {}

  public static DoublePreference warningActiveToInactive =
      new DoublePreference(
          "UI/WarningActiveToInactive", 3.0); // seconds before losing hub to show warning

  public static DoublePreference warningInactiveToActive =
      new DoublePreference(
          "UI/WarningInactiveToActive", 5.0); // seconds before gaining hub to show warning

  public static DoublePreference endgameSlowPulse =
      new DoublePreference(
          "UI/EndgameSlowPulse", 10.0); // seconds remaining to start slow pulse (2Hz)

  public static DoublePreference endgameFastPulse =
      new DoublePreference(
          "UI/EndgameFastPulse", 5.0); // seconds remaining to start fast pulse (5Hz)
}
