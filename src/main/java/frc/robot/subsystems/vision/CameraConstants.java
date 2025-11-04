package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {

    private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final String photonCameraName1 = "CAMERA 1";
    public static final Transform3d photonCameraTransform1 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final PhotonCamera photonCamera1 = new PhotonCamera(photonCameraName1);
    public static final PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform1);

    public static final String photonCameraName2 = "CAMERA 2";
    public static final Transform3d photonCameraTransform2 = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, 0));
    public static final PhotonCamera photonCamera2 = new PhotonCamera(photonCameraName2);
    public static final PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransform2);
}

