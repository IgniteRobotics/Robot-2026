package frc.robot.statemachines;

public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
