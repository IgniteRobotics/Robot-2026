package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class CameraConstants {

  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Front Camera
  public static final String photonCameraName_Front = "FRONT-CAMERA";

  public static final Transform3d photonCameraTransform_Front_STATIC =
      new Transform3d(new Translation3d(-0.18415, 0, 0.7239), new Rotation3d(0, 0, 0));
  // x was 0.3175, y was 0.0706, z was 0.2477

  public static final Transform3d photonCameraTransform_Front_TILTED =
      new Transform3d(
          new Translation3d(0.3124, 0.0706, 0.3543), new Rotation3d(0, Math.toRadians(35), 0));

  public static final PhotonCamera photonCamera_Front = new PhotonCamera(photonCameraName_Front);

  public static final PhotonPoseEstimator photonPoseEstimator_Front =
      new PhotonPoseEstimator(layout, photonCameraTransform_Front_STATIC);

  // Left camera
  public static final String photonCameraName_Left = "LEFT-CAMERA";

  public static final Transform3d photonCameraTransform_Left =
      new Transform3d(
          new Translation3d(-0.2435, 0.3175, 0.43815),
          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(120)));

  public static final PhotonCamera photonCamera_Left = new PhotonCamera(photonCameraName_Left);

  public static final PhotonPoseEstimator photonPoseEstimator_Left =
      new PhotonPoseEstimator(layout, photonCameraTransform_Left);

  // Right camera
  public static final String photonCameraName_Right = "RIGHT-CAMERA";

  public static final Transform3d photonCameraTransform_Right =
      new Transform3d(
          new Translation3d(-0.2435, -0.3175, 0.43815),
          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-120)));

  public static final PhotonCamera photonCamera_Right = new PhotonCamera(photonCameraName_Right);

  public static final PhotonPoseEstimator photonPoseEstimator_Right =
      new PhotonPoseEstimator(layout, photonCameraTransform_Right);

  public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;
}
