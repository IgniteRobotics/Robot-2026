package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class CameraConstants {

  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final String photonCameraName_Front = "FRONT-CAMERA";

  public static final Transform3d photonCameraTransform_Front_STATIC =
      new Transform3d(new Translation3d(0.3175, 0.0706, 0.2477), new Rotation3d(0, 0, 0));

  public static final Transform3d photonCameraTransform_Front_TILTED =
      new Transform3d(
          new Translation3d(0.3124, 0.0706, 0.3543), new Rotation3d(0, Math.toRadians(35), 0));

  public static final PhotonCamera photonCamera_Front = new PhotonCamera(photonCameraName_Front);

  public static final PhotonPoseEstimator photonPoseEstimator_Front =
      new PhotonPoseEstimator(layout, photonCameraTransform_Front_STATIC);

  public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;
  public static final photonEstimator = new PhotonPoseEstimator(AprilTagFieldLayout, photonCameraTransform_Front_STATIC);
  
}
