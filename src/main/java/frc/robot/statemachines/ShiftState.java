package frc.robot.statemachines;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Logged
public class ShiftState {

  private static ShiftState single_instance = null;

  public static enum ActiveState {
    RED_ACTIVE,
    BLUE_ACTIVE,
    BOTH_ACTIVE,
    UNKNOWN
  }

  public static enum Shift {
    AUTONOMOUS,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    NONE
  }

  // Shift timing constants (in seconds of remaining match time)
  public static final double TELEOP_START_TIME =
      140.0; // When teleop starts (20s auto per 2026 REBUILT)
  public static final double TRANSITION_END_TIME = 130.0; // When transition ends
  public static final double SHIFT_1_END_TIME = 105.0; // When shift 1 ends
  public static final double SHIFT_2_END_TIME = 80.0; // When shift 2 ends
  public static final double SHIFT_3_END_TIME = 55.0; // When shift 3 ends
  public static final double SHIFT_4_END_TIME = 30.0; // When shift 4 ends
  public static final double ENDGAME_END_TIME = 0.0; // When endgame ends (match end)

  @Logged(name = "Shift State")
  private ActiveState shiftState = ActiveState.UNKNOWN;

  @Logged(name = "Current Shift")
  private Shift currentShift = Shift.NONE;

  private boolean isFMSConnected = DriverStation.isFMSAttached();

  @Logged(name = "Red Won Auton")
  private boolean redWonAuton = true; // Set to true if red wins auton, false if blue wins auton

  private ShiftState() {}

  public static synchronized ShiftState getInstance() {
    if (single_instance == null) single_instance = new ShiftState();

    return single_instance;
  }

  public ActiveState getShiftState() {
    return shiftState;
  }

  private void updateInternalShiftState() {
    switch (getShiftFromMatchTime()) {
      case AUTONOMOUS, TRANSITION, ENDGAME: // auto, transition, endgame
        shiftState = ActiveState.BOTH_ACTIVE;
        break;
      case SHIFT_1, SHIFT_3: // shift 1 and shift 3
        shiftState = redWonAuton ? ActiveState.BLUE_ACTIVE : ActiveState.RED_ACTIVE;
        break;
      case SHIFT_2, SHIFT_4: // shift 2 and shift 4
        shiftState = redWonAuton ? ActiveState.RED_ACTIVE : ActiveState.BLUE_ACTIVE;
        break;
      default:
        shiftState = ActiveState.UNKNOWN;
    }
  }

  public boolean isOurHubActive() {
    ActiveState shiftState = getShiftState(); // update shiftState before checking
    Alliance alliance = AllianceState.getInstance().getAlliance();

    if (shiftState == ActiveState.BOTH_ACTIVE) return true;
    if (shiftState == ActiveState.UNKNOWN) return false;

    return (shiftState == ActiveState.RED_ACTIVE && alliance == Alliance.Red)
        || (shiftState == ActiveState.BLUE_ACTIVE && alliance == Alliance.Blue);
  }

  public Shift getShiftFromMatchTime() {
    Shift shiftStateInt = Shift.NONE;
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isAutonomousEnabled()) { // Auton
      shiftStateInt = Shift.AUTONOMOUS;
    } else if (DriverStation.isTeleopEnabled()) {
      if (matchTime >= TRANSITION_END_TIME) { // Transition shift
        shiftStateInt = Shift.TRANSITION;
      } else if (matchTime >= SHIFT_1_END_TIME) { // Shift 1
        shiftStateInt = Shift.SHIFT_1;
      } else if (matchTime >= SHIFT_2_END_TIME) { // Shift 2
        shiftStateInt = Shift.SHIFT_2;
      } else if (matchTime >= SHIFT_3_END_TIME) { // Shift 3
        shiftStateInt = Shift.SHIFT_3;
      } else if (matchTime >= SHIFT_4_END_TIME) { // Shift 4
        shiftStateInt = Shift.SHIFT_4;
      } else { // Endgame
        shiftStateInt = Shift.ENDGAME;
      }
    }
    return shiftStateInt;
  }

  /**
   * Returns a formatted countdown string showing time remaining until the current shift ends.
   *
   * <p>Format: "XX.YY" (e.g., "05.25" for 5.25 seconds remaining)
   *
   * @return Countdown string with leading zeros. Returns "00.00" when shift has ended. Returns
   *     "--:--" for NONE/unknown state.
   */
  @Logged(name = "Shift Countdown")
  public String getShiftCountdownString() {
    double matchTime = DriverStation.getMatchTime();
    double shiftEndTime;

    switch (currentShift) {
      case AUTONOMOUS:
        // Autonomous ends when teleop starts (at TELEOP_START_TIME remaining)
        shiftEndTime = TELEOP_START_TIME;
        break;
      case TRANSITION:
        shiftEndTime = TRANSITION_END_TIME;
        break;
      case SHIFT_1:
        shiftEndTime = SHIFT_1_END_TIME;
        break;
      case SHIFT_2:
        shiftEndTime = SHIFT_2_END_TIME;
        break;
      case SHIFT_3:
        shiftEndTime = SHIFT_3_END_TIME;
        break;
      case SHIFT_4:
        shiftEndTime = SHIFT_4_END_TIME;
        break;
      case ENDGAME:
        shiftEndTime = ENDGAME_END_TIME;
        break;
      case NONE:
      default:
        return "--:--";
    }

    double timeRemaining = matchTime - shiftEndTime;
    if (timeRemaining < 0) {
      return "00.00";
    }

    return formatCountdownString(timeRemaining);
  }

  private String formatCountdownString(double timeSeconds) {
    int seconds = (int) timeSeconds;
    int hundredths = (int) ((timeSeconds - seconds) * 100);

    // Clamp hundredths to 0-99 range (handle floating point precision)
    if (hundredths > 99) {
      hundredths = 99;
    }

    return String.format("%02d.%02d", seconds, hundredths);
  }

  public void periodic() {
    // Update FMS connection status
    isFMSConnected = DriverStation.isFMSAttached(); // Update periodically
    currentShift = getShiftFromMatchTime(); // Update current shift based on match time

    if (isFMSConnected) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        redWonAuton = (gameData.charAt(0) == 'R'); // true if red won auton, false if blue won auton
      }
    }
    // Update the internal shiftState based on current match time and redWonAuton
    updateInternalShiftState();
  }
}
