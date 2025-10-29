package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.io.UncheckedIOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionSubsystem {

    private boolean m_seesTarget;
    
    private double m_yawRadians;

    public class TargetInfo{
        private double yaw;

        private PhotonCamera m_camera;

        private int m_tag_Id;

        private Transform3d m_transform3d;

        public TargetInfo(Transform3d transform3d, double yaw, int tag_Id, PhotonCamera camera){
            this.yaw = yaw;
            this.m_camera = camera;
            this.m_transform3d = transform3d;
            this.m_tag_Id = tag_Id;
        }

        public double getYaw() {
            return yaw;
        }

        public Transform3d getTransform3d(){
            return m_transform3d;
        }

        public void setYaw(double yaw) {
            this.yaw = yaw;
        }

        public int getTagId(){
            return this.m_tag_Id;
        }

        public void setTagId(int tag_Id){
            this.m_tag_Id = tag_Id;
        }

        public PhotonCamera getCamera(){
            return m_camera;
        }
    }

    public VisionSubsystem() {
        //TODO: investigate PNP on the co-proc.
    }

    public void periodic(){
        /*lastLeftTransform3d = null;
        lastRightTransform3d = null;
        lrTransformDelta = null;

        //outtake left
        if(DriveState.getInstance().useLeftOuttakeCamera()){
            CameraConstants.photonPoseEstimatorOuttakeLeft.setReferencePose(getPose());
            var outtakeLeftResults = CameraConstants.photonCameraOuttakeLeft.getAllUnreadResults();

            if(!outtakeLeftResults.isEmpty()){
                var latestResult = outtakeLeftResults.get(outtakeLeftResults.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeLeft, latestResult);
            }

            else m_driveState.nullify(CameraConstants.photonCameraOuttakeLeft);

            for(var result: outtakeLeftResults){
                Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorOuttakeLeft.update(result);
                if(!estimatedPose.isEmpty()){
                    calculateVisionMeasurement(estimatedPose.get(), 0);
                }
            }
        }

        //outtake right
        if(DriveState.getInstance().useRightOuttakeCamera()){
            CameraConstants.photonPoseEstimatorOuttakeRight.setReferencePose(getPose());
            var outtakeRightResults = CameraConstants.photonCameraOuttakeRight.getAllUnreadResults();

            if(!outtakeRightResults.isEmpty()){
                var latestResult = outtakeRightResults.get(outtakeRightResults.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraOuttakeRight, latestResult);
            }

            else m_driveState.nullify(CameraConstants.photonCameraOuttakeRight);

            for(var result: outtakeRightResults){
                Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorOuttakeRight.update(result);
                if(!estimatedPose.isEmpty()){
                    calculateVisionMeasurement(estimatedPose.get(), 1);
                }
            }
        }

        //intake
        if(DriveState.getInstance().useIntakeCamera()){
            CameraConstants.photonPoseEstimatorIntake.setReferencePose(getPose());
            var intakeResults = CameraConstants.photonCameraIntake.getAllUnreadResults();

            if(!intakeResults.isEmpty()){
                var latestResult = intakeResults.get(intakeResults.size()-1);
                m_driveState.setLatestPhotonVisionResult(CameraConstants.photonCameraIntake, latestResult);
            }

            else m_driveState.nullify(CameraConstants.photonCameraIntake);

            for(var result: intakeResults){
                Optional<EstimatedRobotPose> estimatedPose = CameraConstants.photonPoseEstimatorIntake.update(result);
                if(!estimatedPose.isEmpty()){
                    calculateVisionMeasurement(estimatedPose.get(), 2);
                }
            }
        }

        if(lastLeftTransform3d != null && lastRightTransform3d != null){
            lrTransformDelta = lastLeftTransform3d.plus(lastRightTransform3d.inverse());
        }*/
    }

    //for multiple targets
    /*public Optional<TargetInfo> seekTargets(int[] ids, PhotonCamera camera){

        if(!DriveState.getInstance().hasPhotonVisionResult(camera)){
            return Optional.empty();
        }

        ArrayList< Optional<PhotonTrackedTarget> > targets = new ArrayList< Optional<PhotonTrackedTarget> >();

        var newResult = m_driveState.getLatestPhotonVisionResult(camera);
        if(newResult != null){
            for (int id : ids) {
                Optional<PhotonTrackedTarget> tempTarget = lookForTarget(newResult, id);
                if(tempTarget.isPresent() && tempTarget.get().getPoseAmbiguity() < CameraConstants.MINIMUM_AMBIGUITY){
                    targets.add(tempTarget);
                }
            }
        }

        if(targets.size() > 0){
            Optional<PhotonTrackedTarget> target = targets.get(0);
            double maxArea = target.get().getArea();
            for(int i = 1; i < targets.size(); i++){
                if(maxArea < targets.get(i).get().getArea()){
                    target = targets.get(i);
                    maxArea = target.get().getArea();
                }
            }

            return Optional.of(new TargetInfo(target.get().getBestCameraToTarget(),
                target.get().getYaw(), target.get().getFiducialId(), camera));
        }
        
        

        if (Robot.isSimulation()){
            return Optional.of(new TargetInfo(new Transform3d(3, 2, 0, new Rotation3d(0, 15, 10)),
            10, 18, CameraConstants.photonCameraOuttakeLeft));
        }

        return Optional.empty();

    }


    //for one target
    public Optional<TargetInfo> seekTargets(int id, PhotonCamera camera){

        if(!DriveState.getInstance().hasPhotonVisionResult(camera)){
            return Optional.empty();
        }

        var newResult = m_driveState.getLatestPhotonVisionResult(camera);
        Optional<PhotonTrackedTarget> target = lookForTarget(newResult, id);

        if(target.isPresent() && target.get().getPoseAmbiguity() < CameraConstants.MINIMUM_AMBIGUITY){
            return Optional.of(new TargetInfo(target.get().getBestCameraToTarget(), target.get().getYaw(), target.get().getFiducialId(), camera));
        }

        if (Robot.isSimulation()){
            return Optional.of(new TargetInfo(new Transform3d(3, 2, 0, new Rotation3d(0, 15, 10)),
            10, 18, CameraConstants.photonCameraOuttakeLeft));
        }
    
        return Optional.empty();
    }

    private Optional<PhotonTrackedTarget> lookForTarget(PhotonPipelineResult result, int targetId){
        if(Robot.isSimulation()){   //TODO: Change values (after TargetId 2 addition values, see class)
            return Optional.of(new PhotonTrackedTarget(10, 15, 1, 0.0, targetId, -1, -1,
                new Transform3d(3, 2, 0, new Rotation3d(0.0, 15, 10)),
                new Transform3d(3, 2, 0, new Rotation3d(0.0, 15, 10)),
             0.0, 
             new ArrayList<TargetCorner>(4), 
             new ArrayList<TargetCorner>(4)
             ));
        }


        for (var target : result.getTargets()){
                if (targetId == target.getFiducialId()){
                    return Optional.of(target) ;
                }
            }


        return Optional.empty();
    }*/

    public void setPipeline(int index){
        CameraConstants.photonCameraOuttakeLeft.setPipelineIndex(index);
        CameraConstants.photonCameraOuttakeRight.setPipelineIndex(index);
    }

    public static boolean contains(final int[] arr, final int key) {
        return Arrays.stream(arr).anyMatch(i -> i == key);
    }

}
