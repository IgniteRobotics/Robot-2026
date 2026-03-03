package frc.robot.statemachines;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.LaunchRequest;

@Logged
public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  AllianceState allianceState = AllianceState.getInstance();
  private LaunchCalculator launchCalculator = LaunchCalculator.getInstance();

  @Logged(name = "Current Launch Request")
  private LaunchRequest currentLaunchRequest = null;

  @Logged(name = "3D Target Pose")
  private Pose3d targetPose3d = Constants.FieldConstants.getHubTarget();

  private LaunchType builderType = LaunchType.MAPPED;

  public LaunchRequest getLaunchRequest() {
    return currentLaunchRequest;
  }

  public void refreshRequest() {
    currentLaunchRequest = launchCalculator.refreshRequest(targetPose3d, builderType);
  }

  public void setTargetPose3d(Pose3d target) {
    this.targetPose3d = target;
  }

  public void setBuilderType(LaunchType builderType) {
    this.builderType = builderType;
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
