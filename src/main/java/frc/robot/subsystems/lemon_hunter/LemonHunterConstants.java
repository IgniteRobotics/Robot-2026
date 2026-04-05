package frc.robot.subsystems.lemon_hunter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class LemonHunterConstants {

  private LemonHunterConstants() {}

  //Camera Constants
  public static final String lemonHunterCameraName = "LEMON-HUNTER";
  public static final PhotonCamera lemonHunterCamera = new PhotonCamera(lemonHunterCameraName);
  
  public static final Distance HUNTER_OFFSET = Meters.of(0.61);
  public static final Distance HUNTER_HEIGHT = Meters.of(0.6223);
  public static final Angle HUNTER_PITCH = Degrees.of(-30);
}
