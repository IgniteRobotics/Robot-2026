package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.LaunchRequest;
import frc.robot.subsystems.shooter.MappedLaunchRequestBuilder;
import frc.robot.subsystems.shooter.ParabolicLaunchRequestBuilder;
import frc.robot.subsystems.shooter.ShooterConstants;

public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  private LaunchCalculator launchCalculator = LaunchCalculator.getInstance();
  private LaunchRequest currentLaunchRequest = null;

  private Pose3d targetPose3d = ShooterConstants.RED_TARGET;
  private LaunchType builderType = LaunchType.PARABOLIC;

  public LaunchRequest getLaunchRequest() {
    return currentLaunchRequest;
  }

  public void refreshRequest() {
    currentLaunchRequest = launchCalculator.refreshRequest(targetPose3d, builderType);
  }

  public void setTargetPose3d(Pose3d target) {
    this.targetPose3d = target;
  }

  public void setBuilderType(LaunchType builderType){
    this.builderType = builderType;
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
