package frc.robot.statemachines;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class DriveState {

    private static DriveState single_instance = null;  

    private ArrayList<VisionMeasurement> visionEstimateList = new ArrayList<VisionMeasurement>();
    
    private Pose2d currentRobotPose = null;

    private DriveState() {
            
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

    public void addVisionEstimate(VisionMeasurement estimate){visionEstimateList.add(estimate);}
    public VisionMeasurement grabVisionEstimate(){
        if(visionEstimateList.isEmpty()) return null;
        else{
            return visionEstimateList.remove(0);
        }
    }

    public void adjustRobotPose(Pose2d newPose){currentRobotPose = newPose;}
    public Pose2d getCurrentRobotPose(){return currentRobotPose;}
    

}