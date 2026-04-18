package frc.robot.subsystems.lemon_hunter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.DriveState;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class LemonHunterSubsystem extends SubsystemBase {

  private static final double LEMON_DIAMETER_M = 0.15;

  private static final double CAMERA_FOCAL_LENGTH_PX = 600.0;

  private static final double CLUSTER_RADIUS_M = 0.5;

  private static final double OVERLAP_THRESHOLD_M = LEMON_DIAMETER_M;

  @Logged(name = "Lemon Hunter/Lemon List", importance = Importance.CRITICAL)
  public List<Pose3d> lemonList = new ArrayList<>();

  public List<Pose3d> bestCluster = new ArrayList<>();

  public List<int[]> overlappingPairs = new ArrayList<>();

  public DriveState driveState = DriveState.getInstance();

  private List<PhotonTrackedTarget> latestTargets = new ArrayList<>();

  @Logged(name = "Lemon Field", importance = Importance.CRITICAL)
  private Field2d lemonField = new Field2d();

  @Logged(name = "Results This Cycle", importance = Importance.CRITICAL)
  private int resultsThisCycle = 0;

  @Logged(name = "Lemons Found This Cycle")
  private int lemonsFoundThisCycle = 0;

  @Logged(name = "Best Cluster Size")
  private int bestClusterSize = 0;

  @Logged(name = "Overlapping Pairs")
  private int overlappingPairCount = 0;

  public LemonHunterSubsystem() {}

  @Override
  public void periodic() {
    lemonList.clear();
    bestCluster.clear();
    overlappingPairs.clear();
    latestTargets.clear();

    List<PhotonPipelineResult> results =
        LemonHunterConstants.lemonHunterCamera.getAllUnreadResults();
    resultsThisCycle = results.size();

    if (results.isEmpty()) {
      lemonsFoundThisCycle = 0;
      bestClusterSize = 0;
      return;
    }

    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    latestTargets.addAll(latestResult.targets);

    Pose2d robotPose = driveState.getCurrentDriveStats().Pose;

    for (var target : latestTargets) {
      Pose3d lemonPose = estimateLemon3dPose(robotPose, target);
      lemonList.add(lemonPose);
    }

    lemonsFoundThisCycle = lemonList.size();

    bestCluster = findLargestCluster(lemonList, CLUSTER_RADIUS_M);
    bestClusterSize = bestCluster.size();

    overlappingPairs = detectOverlaps(bestCluster, OVERLAP_THRESHOLD_M);
    overlappingPairCount = overlappingPairs.size();

    lemonField
        .getObject("Lemon Positions")
        .setPoses(lemonList.stream().map(Pose3d::toPose2d).toList());
    lemonField.getObject("Cluster Centroid Position").setPose(getClusterCentroid(robotPose));
    // SmartDashboard.putData("Hunter Field Data", field);
  }

  public Pose3d estimateLemon3dPose(Pose2d robotPose, PhotonTrackedTarget target) {
    double targetYaw = Math.toRadians(target.getYaw());
    double targetPitch = Math.toRadians(target.getPitch());
    double depthMeters;

    var detectedCorners = target.getDetectedCorners();
    if (detectedCorners != null && detectedCorners.size() >= 2) {
      double minX = detectedCorners.stream().mapToDouble(c -> c.x).min().orElse(0);
      double maxX = detectedCorners.stream().mapToDouble(c -> c.x).max().orElse(1);
      double pixelWidth = maxX - minX;

      if (pixelWidth > 0) {

        depthMeters = (LEMON_DIAMETER_M * CAMERA_FOCAL_LENGTH_PX) / pixelWidth;
      } else {
        depthMeters = pitchBasedDistance(target);
      }
    } else {
      depthMeters = pitchBasedDistance(target);
    }

    double dx_cam = depthMeters * Math.cos(targetPitch) * Math.cos(targetYaw);
    double dy_cam = depthMeters * Math.cos(targetPitch) * Math.sin(targetYaw);
    double dz_cam = depthMeters * Math.sin(targetPitch);

    dx_cam += LemonHunterConstants.HUNTER_OFFSET.in(Meters);

    double heading = robotPose.getRotation().getRadians();
    double dx_field = dx_cam * Math.cos(heading) - dy_cam * Math.sin(heading);
    double dy_field = dx_cam * Math.sin(heading) + dy_cam * Math.cos(heading);
    double dz_field = dz_cam;

    return new Pose3d(
        robotPose.getX() + dx_field,
        robotPose.getY() + dy_field,
        dz_field,
        new Rotation3d(0, -targetPitch, targetYaw));
  }

  private double pitchBasedDistance(PhotonTrackedTarget target) {
    return PhotonUtils.calculateDistanceToTargetMeters(
        LemonHunterConstants.HUNTER_HEIGHT.in(Meters),
        0,
        LemonHunterConstants.HUNTER_PITCH.in(Radians),
        Math.toRadians(target.getPitch()));
  }

  public List<Pose3d> findLargestCluster(List<Pose3d> lemons, double radiusM) {
    if (lemons.isEmpty()) return new ArrayList<>();

    int n = lemons.size();
    int[] clusterID = new int[n];
    for (int i = 0; i < n; i++) clusterID[i] = i;

    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (distance3d(lemons.get(i), lemons.get(j)) <= radiusM) {

          int oldID = clusterID[j];
          int newID = clusterID[i];
          for (int k = 0; k < n; k++) {
            if (clusterID[k] == oldID) clusterID[k] = newID;
          }
        }
      }
    }

    int bestID = -1;
    int bestCount = 0;
    for (int i = 0; i < n; i++) {
      int count = 0;
      for (int j = 0; j < n; j++) {
        if (clusterID[j] == clusterID[i]) count++;
      }
      if (count > bestCount) {
        bestCount = count;
        bestID = clusterID[i];
      }
    }

    // Collect only lemons in the largest cluster
    List<Pose3d> result = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      if (clusterID[i] == bestID) result.add(lemons.get(i));
    }
    return result;
  }

  public List<int[]> detectOverlaps(List<Pose3d> lemons, double thresholdM) {
    List<int[]> pairs = new ArrayList<>();
    for (int i = 0; i < lemons.size(); i++) {
      for (int j = i + 1; j < lemons.size(); j++) {
        if (distance3d(lemons.get(i), lemons.get(j)) <= thresholdM) {
          pairs.add(new int[] {i, j});
        }
      }
    }
    return pairs;
  }

  public Pose3d getNearestLemonInCluster(Pose2d robotPose) {
    if (bestCluster.isEmpty()) return null;

    Pose3d nearest = null;
    double minDist = Double.MAX_VALUE;

    for (Pose3d lemon : bestCluster) {
      double dist = Math.hypot(lemon.getX() - robotPose.getX(), lemon.getY() - robotPose.getY());
      if (dist < minDist) {
        minDist = dist;
        nearest = lemon;
      }
    }
    return nearest;
  }

  public Pose2d getClusterCentroid(Pose2d robotPose) {
    if (bestCluster.isEmpty()) return null;

    double sumX = 0, sumY = 0;
    for (Pose3d lemon : bestCluster) {
      sumX += lemon.getX();
      sumY += lemon.getY();
    }
    double cx = sumX / bestCluster.size();
    double cy = sumY / bestCluster.size();

    double angle = Math.atan2(cy - robotPose.getY(), cx - robotPose.getX());
    return new Pose2d(cx, cy, new Rotation2d(angle));
  }

  private double distance3d(Pose3d a, Pose3d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    double dz = a.getZ() - b.getZ();
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}
