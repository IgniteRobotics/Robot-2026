package frc.robot.statemachines;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

public class DriveState {

  private static DriveState single_instance = null;

  private HashMap<Integer, ConcurrentLinkedQueue<VisionMeasurement>> concurrentQueueMap;

  private SwerveDriveState previousDriveStats = new SwerveDriveState();
  private SwerveDriveState currentDriveStats = new SwerveDriveState();

  private DriveState() {
    concurrentQueueMap = new HashMap<Integer, ConcurrentLinkedQueue<VisionMeasurement>>();
    concurrentQueueMap.put(1, new ConcurrentLinkedQueue<VisionMeasurement>());
    concurrentQueueMap.put(2, new ConcurrentLinkedQueue<VisionMeasurement>());
    concurrentQueueMap.put(3, new ConcurrentLinkedQueue<VisionMeasurement>());
  }

  public static synchronized DriveState getInstance() {
    if (single_instance == null) single_instance = new DriveState();

    return single_instance;
  }

  public Alliance getAlliance() {
    return DriverStation.isDSAttached() ? DriverStation.getAlliance().get() : Alliance.Blue;
  }

  public void addVisionEstimate(VisionMeasurement estimate, int camerasUsed) {
    ConcurrentLinkedQueue<VisionMeasurement> tempPointer = concurrentQueueMap.get(camerasUsed);
    tempPointer.add(estimate);
  }

  public ArrayList<VisionMeasurement> grabVisionEstimateList(int camerasUsed) {
    ConcurrentLinkedQueue<VisionMeasurement> tempPointer = concurrentQueueMap.get(camerasUsed);
    ArrayList<VisionMeasurement> dataToExport = new ArrayList<VisionMeasurement>();
    int exportSize = tempPointer.size();
    try {
      for (int i = 0; i < exportSize; i++) {
        dataToExport.add(tempPointer.remove());
      }
    }
    // TODO add to log
    catch (Exception e) {
      System.out.println("EXCEPTION IN grabVisionEstimateList!");
      System.out.println(e.getMessage());
    }
    return dataToExport;
  }

  public void adjustCurrentDriveStats(SwerveDriveState newStats) {
    previousDriveStats = currentDriveStats;
    currentDriveStats = newStats;
  }

  public boolean hasDriveStats() {
    return currentDriveStats != null;
  }

  public SwerveDriveState getCurrentDriveStats() {
    return currentDriveStats;
  }

  public SwerveDriveState getPreviousDriveStats() {
    return previousDriveStats;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getCurrentDriveStats().Speeds, getCurrentDriveStats().Pose.getRotation());
  }
}
