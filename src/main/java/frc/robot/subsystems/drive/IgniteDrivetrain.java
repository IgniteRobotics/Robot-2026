package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;

public class IgniteDrivetrain extends CommandSwerveDrivetrain
{
    private DriveState driveState = DriveState.getInstance();
    public IgniteDrivetrain(){
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    @Override
    public void periodic(){
        super.periodic();
        for(VisionMeasurement estimate : driveState.grabVisionEstimateList(CameraConstants.photonCameraName1)){
            addVisionMeasurement(estimate.getEstimatedPose().estimatedPose.toPose2d(), estimate.getTimestamp(), estimate.getTrust());
        }
        for(VisionMeasurement estimate : driveState.grabVisionEstimateList(CameraConstants.photonCameraName2)){
            addVisionMeasurement(estimate.getEstimatedPose().estimatedPose.toPose2d(), estimate.getTimestamp(), estimate.getTrust());
        }
        driveState.adjustRobotPose(getPose());
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }
}
