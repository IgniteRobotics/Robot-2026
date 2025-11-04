package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
/** Add your docs here. */
public class IgniteDrivetrain extends CommandSwerveDrivetrain
{
    private DriveState driveState = DriveState.getInstance();
    public IgniteDrivetrain(){
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    @Override
    public void periodic(){
        super.periodic();
        do{
            VisionMeasurement estimate = driveState.grabVisionEstimate();
            if(estimate == null)break;
            this.addVisionMeasurement(estimate.getEstimatedPose().estimatedPose.toPose2d(), estimate.getTimestamp(), estimate.getTrust());
        }while(true);
        driveState.adjustRobotPose(getPose());
    }

    public Pose2d getPose(){
        return this.getState().Pose;
    }
}
