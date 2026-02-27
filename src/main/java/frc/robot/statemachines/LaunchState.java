package frc.robot.statemachines;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.MappedLaunchRequestBuilder;
import frc.robot.subsystems.shooter.ParabolicLaunchRequestBuilder;

public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchCalculator currentCalculator = null;

  private LaunchRequest currentRequest = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  public void activateCalculator(Pose3d target, LaunchType calculatorType) {
    if (currentCalculator != null) return;

    if (calculatorType == LaunchType.PARABOLIC)
      currentCalculator = new LaunchCalculator(target, new ParabolicLaunchRequestBuilder());
    else if (calculatorType == LaunchType.MAPPED)
      currentCalculator = new LaunchCalculator(target, new MappedLaunchRequestBuilder());

    refreshRequest();
  }

  public void deactivateCalculator() {
    currentCalculator = null;
  }

  public boolean isActivated() {
    return currentCalculator != null;
  }

  public LaunchRequest getLaunchRequest() {
    if (!isActivated()) return null;

    if (currentRequest.getTimestamp() + 0.02 < Utils.getCurrentTimeSeconds()) refreshRequest();
    return currentRequest;
  }

  private void refreshRequest() {
    currentRequest = currentCalculator.createLaunchRequest();
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
