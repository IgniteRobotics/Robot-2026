package frc.robot.statemachines;

public class HunterState {

  private static HunterState single_instance = null;

  private HunterState() {}

  public static synchronized HunterState getInstance() {
    if (single_instance == null) single_instance = new HunterState();
    return single_instance;
  }
}
