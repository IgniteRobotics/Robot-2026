package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.statemachines.LaunchCalculator.LaunchRequest;
import frc.robot.subsystems.shooter.MappedLaunchRequestBuilder;
import frc.robot.subsystems.shooter.ParabolicLaunchRequestBuilder;

public class LaunchState {
  private static LaunchState single_instance = null;

  private LaunchCalculator currentCalculator = null;

  private LaunchState() {}

  public static synchronized LaunchState getInstance() {
    if (single_instance == null) single_instance = new LaunchState();
    return single_instance;
  }

  public void activateCalculator(Pose3d target, LaunchType calculatorType){
    if(calculatorType == LaunchType.PARABOLIC) currentCalculator = new LaunchCalculator(target, new ParabolicLaunchRequestBuilder());
    else if(calculatorType == LaunchType.MAPPED) currentCalculator = new LaunchCalculator(target, new MappedLaunchRequestBuilder());
  }

  public void deactivateCalculator(){
    currentCalculator = null;
  }
  
  public boolean isActivated(){return currentCalculator != null;}

  public LaunchRequest getLaunchRequest(){
    if(!isActivated()) return null;
    else return currentCalculator.createLaunchRequest();
  }

  public enum LaunchType {
    PARABOLIC,
    MAPPED
  }
}
