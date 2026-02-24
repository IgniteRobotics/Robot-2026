package frc.robot.statemachines;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  @Logged(name = "Shift State")
  private ActiveState shiftState = ActiveState.UNKNOWN;

  private boolean isFMSConnected = DriverStation.isFMSAttached();

  @Logged(name = "Red Won Auton")
  private boolean redWonAuton = true; // Set to true if red wins auton, false if blue wins auton

  private ShiftState() {}

  public static synchronized ShiftState getInstance() {
    if (single_instance == null) single_instance = new ShiftState();

    return single_instance;
  }

  public ActiveState getShiftState() {
    if (isFMSConnected) {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0)
        redWonAuton =
            gameData.charAt(0) == 'R'; // Set to true if red won auton, false if blue won auton
    }

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

    return shiftState;
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
      if (matchTime >= 130) { // Transition shift
        shiftStateInt = Shift.TRANSITION;
      } else if (matchTime >= 105) { // Shift 1
        shiftStateInt = Shift.SHIFT_1;
      } else if (matchTime >= 80) { // Shift 2
        shiftStateInt = Shift.SHIFT_2;
      } else if (matchTime >= 55) { // Shift 3
        shiftStateInt = Shift.SHIFT_3;
      } else if (matchTime >= 30) { // Shift 4
        shiftStateInt = Shift.SHIFT_4;
      } else { // Endgame
        shiftStateInt = Shift.ENDGAME;
      }
    }
    return shiftStateInt;
  }
}
