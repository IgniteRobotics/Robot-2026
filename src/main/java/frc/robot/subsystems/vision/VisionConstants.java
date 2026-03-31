package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import java.util.HashMap;
import org.photonvision.PhotonCamera;

public class VisionConstants {

  // field layout
  public static final AprilTagFieldLayout FIELD_LAYOUT = Constants.FieldConstants.layout;

  // Front Camera
  public static final String photonCameraName_Front = "FRONT-CAMERA";
  public static final PhotonCamera photonCamera_Front = new PhotonCamera(photonCameraName_Front);
  //AI Model Camera 
  public static final  String photonCameraname_AI = "lemon_hunter";
  public static final PhotonCamera photonCamera_AI = new PhotonCamera(photonCameraname_AI);
  


  // Left camera
  public static final String photonCameraName_Left = "LEFT-CAMERA";
  public static final PhotonCamera photonCamera_Left = new PhotonCamera(photonCameraName_Left);

  // Right camera
  public static final String photonCameraName_Right = "RIGHT-CAMERA";
  public static final PhotonCamera photonCamera_Right = new PhotonCamera(photonCameraName_Right);

  // TODO: TUNE
  public static final double XY_STD_DEV_COEFFICIENT = 0.01;
  public static final double THETA_STD_DEV_COEFFICIENT = 0.03;

  // Center of Robot to Camera Transform
  public static final HashMap<String, Transform3d> cameraTransformMap =
      new HashMap<String, Transform3d>() {
        {
          put(
              photonCameraName_Front,
              new Transform3d(
                  new Translation3d(-0.1638, 0, 0.7192),
                  new Rotation3d(0, Math.toRadians(-20), 0)));
          put(
              photonCameraName_Left,
              new Transform3d(
                  new Translation3d(-0.2680, 0.3164, 0.451),
                  new Rotation3d(0, Math.toRadians(-20), Math.toRadians(120))));
          put(
              photonCameraName_Right,
              new Transform3d(
                  new Translation3d(-0.2680, -0.3164, 0.451),
                  new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-120))));
        }
      };

  public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.25;
  public static final double OMEGA_PENALTY = 0.5;
}
