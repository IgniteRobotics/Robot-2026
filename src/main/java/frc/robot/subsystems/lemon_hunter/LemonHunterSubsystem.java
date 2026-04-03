package frc.robot.subsystems.lemon_hunter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;

public class LemonHunterSubsystem extends SubsystemBase{
    
    public LemonHunterSubsystem(){}

    public ArrayList<Pose2d> lemonList = new ArrayList<Pose2d>();

    @Logged(name = "Lemon Hunter/Results This Cycle", importance = Importance.CRITICAL)
    private int resultsThisCycle = 0;
    
    @Logged(name = "Lemon Hunter/Lemons Found This Cycle")
    private int lemonsFoundThisCycle = 0;

    @Override
    public void periodic(){

        lemonList.clear();
        List<PhotonPipelineResult> lemonHunterResults = LemonHunterConstants.lemonHunterCamera.getAllUnreadResults();
        resultsThisCycle = lemonHunterResults.size();
        for(var result : lemonHunterResults){
            for(var target : result.targets){

            }
        }
        lemonsFoundThisCycle = lemonList.size();
    }

    public void locateLemon(PhotonTrackedTarget target){
        
    }
}
