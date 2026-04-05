package frc.robot.subsystems.lemon_hunter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.vision.VisionConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

public class LemonHunterSubsystem extends SubsystemBase{
    
    public LemonHunterSubsystem(){}

    public ArrayList<Pose2d> lemonList = new ArrayList<Pose2d>();

    public DriveState driveState = DriveState.getInstance();

    @Logged(name = "Lemon Hunter/Results This Cycle", importance = Importance.CRITICAL)
    private int resultsThisCycle = 0;
    
    @Logged(name = "Lemon Hunter/Lemons Found This Cycle")
    private int lemonsFoundThisCycle = 0;

    @Override
    public void periodic(){

        lemonList.clear();
        List<PhotonPipelineResult> lemonHunterResults = LemonHunterConstants.lemonHunterCamera.getAllUnreadResults();
        resultsThisCycle = lemonHunterResults.size();

        if(!lemonHunterResults.isEmpty()){
            //choose the latest result
            PhotonPipelineResult latestResult = lemonHunterResults.get(lemonHunterResults.size()-1);

            for(var target : latestResult.targets){
                //uses the heights and pitches of the camera and target to calculate magnitude
                double cameraToTargetDistance = PhotonUtils.calculateDistanceToTargetMeters(
                    LemonHunterConstants.HUNTER_HEIGHT.in(Meters), 0,  LemonHunterConstants.HUNTER_PITCH.in(Radians), Math.toRadians(target.pitch));
                
                //uses the pitch of the target and the magnitude to calculate the translation vector
                Translation2d cameraToTarget2dTransform = PhotonUtils.estimateCameraToTargetTranslation(cameraToTargetDistance, new Rotation2d(Math.toRadians(target.yaw)));

                //adds the camera's offset from the robot's center to the x-coordinate fo the translation vector
                cameraToTarget2dTransform = cameraToTarget2dTransform.plus(new Translation2d(LemonHunterConstants.HUNTER_OFFSET.in(Meters), 0));

                //current robot pose
                Pose2d currentRobotPose = driveState.getCurrentDriveStats().Pose;
                
                //calculates the position of the lemon using the robot's position
                Pose2d lemonPose = new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), new Rotation2d())
                    .plus(new Transform2d(cameraToTarget2dTransform.rotateBy(currentRobotPose.getRotation()), new Rotation2d()));
                
                lemonList.add(lemonPose);
            }
        }
        lemonsFoundThisCycle = lemonList.size();
    }

    public void locateLemon(PhotonTrackedTarget target){
        
    }
}
