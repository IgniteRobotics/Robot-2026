package frc.robot.subsystems.ui;

public final class UIConstants {

  private UIConstants() {}

  // Color hex codes for Elastic dashboard
  public static final String COLOR_GREEN = "#00FF00"; // OK to shoot
  public static final String COLOR_RED = "#FF0000"; // NOT OK to shoot
  public static final String COLOR_YELLOW = "#FFFF00"; // Warning period
  public static final String COLOR_BLACK = "#000000"; // Off state (for flashing)

  // Flash timing (in periodic cycles @ 50Hz)
  public static final int FAST_FLASH_DURATION_CYCLES = 50; // 1 second @ 50Hz
  public static final int FAST_FLASH_TOGGLE_CYCLES = 5; // 5Hz (on 5 cycles, off 5 cycles)

  public static final int YELLOW_PULSE_TOGGLE_CYCLES = 12; // ~2Hz (on 12 cycles, off 13 cycles)

  // Rumble patterns
  public static final double RUMBLE_WARNING_INTENSITY = 0.5; // 50% intensity during warning
  public static final int RUMBLE_WARNING_TOGGLE_CYCLES = YELLOW_PULSE_TOGGLE_CYCLES; // 2Hz pulse

  public static final double RUMBLE_TRANSITION_INTENSITY = 0.5; // 50% intensity during transition
  public static final int RUMBLE_TRANSITION_TOGGLE_CYCLES = FAST_FLASH_TOGGLE_CYCLES; // 5Hz pulse

  public static final double RUMBLE_MANUAL_INTENSITY = 1.0; // 100% continuous

  // Endgame rumble patterns (takes priority over shift rumble)
  public static final double RUMBLE_ENDGAME_INTENSITY =
      0.75; // 75% intensity during endgame warnings
  public static final int RUMBLE_ENDGAME_SLOW_TOGGLE_CYCLES =
      YELLOW_PULSE_TOGGLE_CYCLES; // 2Hz pulse
  public static final int RUMBLE_ENDGAME_FAST_TOGGLE_CYCLES = FAST_FLASH_TOGGLE_CYCLES; // 5Hz pulse
}
