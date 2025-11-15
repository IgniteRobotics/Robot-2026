package frc.robot.statemachines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DriveState {

    private static DriveState single_instance = null; 
     
    private HashMap<String, ConcurrentLinkedQueue<VisionMeasurement>> concurrentQueueMap;
    
    private SwerveDriveState currentDriveStats = null;

    private DriveState() {
        concurrentQueueMap = new HashMap<String, ConcurrentLinkedQueue<VisionMeasurement>>();
        concurrentQueueMap.put(CameraConstants.photonCameraName1, new ConcurrentLinkedQueue<VisionMeasurement>());
        concurrentQueueMap.put(CameraConstants.photonCameraName2, new ConcurrentLinkedQueue<VisionMeasurement>());
    }

    public static synchronized DriveState getInstance()
    {
        if (single_instance == null)
            single_instance = new DriveState();
        
        return single_instance;
    }

    public Alliance getAlliance(){
        return DriverStation.isDSAttached() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }

    public void addVisionEstimate(VisionMeasurement estimate, String cameraName){
        ConcurrentLinkedQueue<VisionMeasurement> tempPointer =  concurrentQueueMap.get(cameraName);
        tempPointer.add(estimate);
    }

    public ArrayList<VisionMeasurement> grabVisionEstimateList(String cameraName){
        ConcurrentLinkedQueue<VisionMeasurement> tempPointer = concurrentQueueMap.get(cameraName);
        ArrayList<VisionMeasurement> dataToExport = new ArrayList<VisionMeasurement>();
        int exportSize = tempPointer.size();
        try{
            for(int i = 0; i < exportSize; i++){
                dataToExport.add(tempPointer.remove());
            }
        }
        //TODO add to log
        catch(Exception e){
            System.out.println("EXCEPTION IN grabVisionEstimateList!");
            System.out.println(e.getMessage());
        }
        return dataToExport;
    }

    public void adjustCurrentDriveStats(SwerveDriveState newStats){currentDriveStats = newStats;}
    public boolean hasDriveStats(){return currentDriveStats != null;}
    public SwerveDriveState getCurrentDriveStats(){return currentDriveStats;}
}