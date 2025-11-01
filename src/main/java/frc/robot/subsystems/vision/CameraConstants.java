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

    public static final String photonCameraNameOuttakeLeft = "OUTTAKE_LEFT";
    public static final Transform3d photonCameraTransformOuttakeLeft= new Transform3d(new Translation3d(-.1277, 0.2667, .4964), new Rotation3d(0.0, -15 / 180.0 * Math.PI, Math.PI));
    public static final PhotonCamera photonCameraOuttakeLeft = new PhotonCamera(photonCameraNameOuttakeLeft);
    public static final PhotonPoseEstimator photonPoseEstimatorOuttakeLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransformOuttakeLeft);

    // from pose delta
    //Rotation:
    //x = -0.961, y = -0.013, z = 0.277
    public static final String photonCameraNameOuttakeRight = "OUTTAKE_RIGHT";
    public static final Transform3d photonCameraTransformOuttakeRight = new Transform3d(new Translation3d(-.1277, -0.2667, .4964), new Rotation3d(0, (-15 / 180.0 * Math.PI), Math.PI));
    public static final PhotonCamera photonCameraOuttakeRight = new PhotonCamera(photonCameraNameOuttakeRight);
    public static final PhotonPoseEstimator photonPoseEstimatorOuttakeRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraTransformOuttakeRight);

    public static final String photonCameraNameIntake = "INTAKE";
    public static final Transform3d photonCameraTransformIntake = new Transform3d(new Translation3d(.343, -0.271, .239), new Rotation3d(0.0, 15 / 180.0 * Math.PI,  0));
    public static final PhotonCamera photonCameraIntake = new PhotonCamera(photonCameraNameIntake);
    public static final PhotonPoseEstimator photonPoseEstimatorIntake = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraConstants.photonCameraTransformIntake);


    public static final PhotonPoseEstimator[] allPhotonPoseEstimators= {photonPoseEstimatorOuttakeLeft, photonPoseEstimatorOuttakeRight, photonPoseEstimatorIntake};
    
    public static final Integer[] IGNORED_POSE_TARGETS = {50,51};
    public static final HashMap<String, Double> offsetToBumper = new HashMap<String, Double>(){{
        put("OUTTAKE_LEFT", 0.3429);
        put("OUTTAKE_RIGHT", 0.3429);
    }};

    public static final double Y_LEFT_CORAL_OFFSET_METERS = -0.16;
    public static final double Y_RIGHT_CORAL_OFFSET_METERS = 0.18;

    public static final double X_OFFSET_METERS = 0.37;

    //sets of tag ids for targeting
    public static final int[] RED_HUMAN_PLAYER_TAGS = {1,2};
    public static final int[] BLUE_HUMAN_PLAYER_TAGS = {12,13};
    public static final int[] RED_REEF_TAGS = {6,7,8,9,10,11};
    public static final int[] BLUE_REEF_TAGS = {17,18,19,20,21,22};
    public static final int[] RED_PROCESSOR_TAGS = {3};
    public static final int[] BLUE_PROCESSOR_TAGS = {16};
    public static final int[] RED_BARGE_TAGS = {5,15};
    public static final int[] BLUE_BARGE_TAGS = {4,14};

    public static final double CORAL_REEF_STOP_DISTANCE_METERS = 0.359;
    
    public static final double ALGAE_REEF_STOP_DISTANCE_METERS = 1.0;
    
    public static final double ALGAE_PROCESSOR_STOP_DISTANCE_METERS = 0.5;

    public static final double MINIMUM_AMBIGUITY = 0.7;
}

