package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.robot.statemachines.DriveState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

    private DriveState driveState = DriveState.getInstance();
    public class VisionMeasurement{
        private EstimatedRobotPose estimatedPose;
        private double timestamp;
        private Vector<N3> trustValues;
        
        public VisionMeasurement(EstimatedRobotPose pose, double time, Vector<N3> trust){
            estimatedPose = pose;
            timestamp = time;
            trustValues = trust;
        }

        public EstimatedRobotPose getEstimatedPose(){return estimatedPose;}
        public double getTimestamp(){return timestamp;}
        public Vector<N3> getTrust(){return trustValues;}
    }

    public VisionSubsystem() {

    }

    public void periodic(){

        //camera1
        CameraConstants.photonPoseEstimator1.setReferencePose(driveState.getCurrentRobotPose());
        var camera1Results = CameraConstants.photonCamera1.getAllUnreadResults();

        for(var result: camera1Results){
            Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimator1.update(result);
            if(!estimatedPose.isEmpty()){
                calculateVisionMeasurement(estimatedPose.get());
            }
        }

        CameraConstants.photonPoseEstimator2.setReferencePose(driveState.getCurrentRobotPose());
        var camera2Results = CameraConstants.photonCamera2.getAllUnreadResults();

        for(var result: camera2Results){
            Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimator2.update(result);
            if(!estimatedPose.isEmpty()){
                calculateVisionMeasurement(estimatedPose.get());
            }
        }
    }

    public void setPipeline(int index){
        CameraConstants.photonCamera1.setPipelineIndex(index);
        CameraConstants.photonCamera1.setPipelineIndex(index);
    }

    public void calculateVisionMeasurement(EstimatedRobotPose pose){
        double xyStds = 0.5;
        double thetaStd = 0.5;
        driveState.addVisionEstimate(new VisionMeasurement(pose, Utils.getCurrentTimeSeconds(), VecBuilder.fill(xyStds, xyStds, thetaStd)));
    }
    
}
