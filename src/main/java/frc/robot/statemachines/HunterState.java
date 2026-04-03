package frc.robot.statemachines;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.lemon_hunter.LemonHunterConstants;

public class HunterState {

  private static HunterState single_instance = null;
  private Transform3d robotToHunterTransform = LemonHunterConstants.lemonHunterBaseTransform;

  private HunterState() {}

  public static synchronized HunterState getInstance() {
    if (single_instance == null) single_instance = new HunterState();
    return single_instance;
  }

  public Transform3d adjustIntakeExtenstion(Angle rotations){
    return robotToHunterTransform.plus(new Transform3d(
        LemonHunterConstants.movementPerIntakeExtenstionRotation.times(rotations.in(Rotations)),
        new Rotation3d(0, 0, 0))
    );
  }

  public Transform3d getRobotToHunterTransform(){
    return robotToHunterTransform;
  }
}
