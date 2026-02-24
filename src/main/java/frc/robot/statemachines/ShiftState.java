package frc.robot.statemachines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShiftState {

  private static ShiftState single_instance = null;

  public static enum ShiftStateEnum {
    RED_ACTIVE,
    BLUE_ACTIVE,
    BOTH_ACTIVE,
    UNKNOWN
  }

  private ShiftStateEnum shiftState = ShiftStateEnum.UNKNOWN;
  private boolean isFMSConnected = DriverStation.isFMSAttached();

  private boolean redWonAuton = true; // Set to true if red wins auton, false if blue wins auton

  private ShiftState() {
    SmartDashboard.putString("Shift State", shiftState.toString());
    SmartDashboard.putBoolean("Red Won Auton?", redWonAuton);
  }

  public static synchronized ShiftState getInstance() {
    if (single_instance == null) single_instance = new ShiftState();

    return single_instance;
  }

  public ShiftStateEnum getShiftState() {
    if (!isFMSConnected) { // FMS not connected, use match time to determine shift state
      switch (getShiftFromMatchTime()) {
        case 0, 1, 6:
          shiftState = ShiftStateEnum.BOTH_ACTIVE;
          break;
        case 2, 4:
          shiftState = ShiftStateEnum.BLUE_ACTIVE;
        case 3, 5:
          shiftState = ShiftStateEnum.RED_ACTIVE;
          break;
        default:
          shiftState = ShiftStateEnum.UNKNOWN;
      }
    } else {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        switch (gameData.charAt(0)) {
          case 'B':
            shiftState = ShiftStateEnum.BLUE_ACTIVE;
            break;
          case 'R':
            shiftState = ShiftStateEnum.RED_ACTIVE;
            break;
          default:
            shiftState = ShiftStateEnum.BOTH_ACTIVE;
        }
      } else shiftState = ShiftStateEnum.UNKNOWN;
    }

    if (!redWonAuton) { // Invert shift state if blue won auton (manually set)
      if (shiftState == ShiftStateEnum.BLUE_ACTIVE) shiftState = ShiftStateEnum.RED_ACTIVE;
      else if (shiftState == ShiftStateEnum.RED_ACTIVE) shiftState = ShiftStateEnum.BLUE_ACTIVE;
    }

    SmartDashboard.putString("Shift State", shiftState.toString());
    return shiftState;
  }

  public boolean isOurHubActive() {
    ShiftStateEnum shiftState = getShiftState(); // update shiftState before checking
    Alliance alliance = AllianceState.getInstance().getAlliance();

    if (shiftState == ShiftStateEnum.BOTH_ACTIVE) return true;
    if (shiftState == ShiftStateEnum.UNKNOWN) return false;

    return (shiftState == ShiftStateEnum.RED_ACTIVE && alliance == Alliance.Red)
        || (shiftState == ShiftStateEnum.BLUE_ACTIVE && alliance == Alliance.Blue);
  }

  /*
   * Determines shift state based on match time. Used when FMS is not connected (testing, practice bot, etc.)
   * Shift states are:
   *   0: Autonomous (both active)
   *   1: Transition (both active)
   *   2: Shift 1
   *   3: Shift 2
   *   4: Shift 3
   *   5: Shift 4
   *   6: Endgame (both active)
   */
  public int getShiftFromMatchTime() {
    int shiftStateInt = -1;
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isAutonomousEnabled()) { // Auton
      shiftStateInt = 0;
    } else if (DriverStation.isTeleopEnabled()) {
      if (matchTime >= 130) { // Transition shift
        shiftStateInt = 1;
      } else if (matchTime >= 105) { // Shift 1
        shiftStateInt = 2;
      } else if (matchTime >= 80) { // Shift 2
        shiftStateInt = 3;
      } else if (matchTime >= 55) { // Shift 3
        shiftStateInt = 4;
      } else if (matchTime >= 30) { // Shift 4
        shiftStateInt = 5;
      } else { // Endgame
        shiftStateInt = 6;
      }
    } else shiftStateInt = -1; // none (teleop disabled)
    return shiftStateInt;
  }
}
