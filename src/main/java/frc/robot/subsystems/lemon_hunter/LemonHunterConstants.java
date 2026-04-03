package frc.robot.subsystems.lemon_hunter;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LemonHunterConstants {

  private LemonHunterConstants() {}

  //Camera Constants
  public static final String lemonHunterCameraName = "LEMON-HUNTER";
  public static final PhotonCamera lemonHunterCamera = new PhotonCamera(lemonHunterCameraName);
  
  //Base Transform (when intake is pulled in), actual transform updating as intake moves
  public static final Transform3d lemonHunterBaseTransform = new Transform3d(
                  new Translation3d(0.3429, 0, 0.692),
                  new Rotation3d(0, Math.toRadians(30), 0));
}
