package frc.robot.statemachines;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Queue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DriveState {

    private static DriveState single_instance = null; 
     
    private final int maxMeasurementsPerCamera = 8;
    private HashMap<String, ArrayList<VisionMeasurement>> controlledListMap;
    
    private Pose2d currentRobotPose = null;

    private DriveState() {
        controlledListMap.put(CameraConstants.photonCameraName1, new ArrayList<VisionMeasurement>());
        controlledListMap.put(CameraConstants.photonCameraName2, new ArrayList<VisionMeasurement>());
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
        ArrayList<VisionMeasurement> tempPointer = controlledListMap.get(cameraName);
        if(tempPointer.size() == maxMeasurementsPerCamera){
            tempPointer.remove(maxMeasurementsPerCamera-1);
        }
        tempPointer.add(0, estimate);
    }

    public ArrayList<VisionMeasurement> grabVisionEstimateList(String cameraName){
        ArrayList<VisionMeasurement> tempPointer = controlledListMap.get(cameraName);
        controlledListMap.replace(cameraName, new ArrayList<VisionMeasurement>());
        return tempPointer;
    }

    public void adjustRobotPose(Pose2d newPose){currentRobotPose = newPose;}
    public Pose2d getCurrentRobotPose(){return currentRobotPose;}
}