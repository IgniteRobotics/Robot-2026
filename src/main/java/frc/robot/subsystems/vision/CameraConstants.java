package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CameraConstants {

  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Front Camera
  public static final String photonCameraName_Front = "FRONT-CAMERA";

  public static final Transform3d photonCameraTransform_Front =
      new Transform3d(
          new Translation3d(-0.1651, 0, 0.7191), new Rotation3d(0, Math.toRadians(-20), 0));

  public static final PhotonCamera photonCamera_Front = new PhotonCamera(photonCameraName_Front);

  public static final PhotonPoseEstimator photonPoseEstimator_Front =
      new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform_Front);

  // Left camera
  public static final String photonCameraName_Left = "LEFT-CAMERA";

  public static final Transform3d photonCameraTransform_Left =
      new Transform3d(
          new Translation3d(-0.2435, 0.3174, 0.3908),
          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(120)));

  public static final PhotonCamera photonCamera_Left = new PhotonCamera(photonCameraName_Left);

  public static final PhotonPoseEstimator photonPoseEstimator_Left =
      new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform_Left);

  // Right camera
  public static final String photonCameraName_Right = "RIGHT-CAMERA";

  public static final Transform3d photonCameraTransform_Right =
      new Transform3d(
          new Translation3d(-0.2435, -0.3175, 0.3908),
          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-120)));

  public static final PhotonCamera photonCamera_Right = new PhotonCamera(photonCameraName_Right);

  public static final PhotonPoseEstimator photonPoseEstimator_Right =
      new PhotonPoseEstimator(
          layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform_Right);

  public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;
}
