package frc.robot.subsystems.vision;
import edu.wpi.first.math.MathUtil;
import org.photonvision.PhotonCamera;
public class LemonCount{

    private var data = photonCamera_AI.getLatestResult();
    boolean hasTarget = data.hasTargets();
    List<PhotonTrackedTarget> inferences = result.getTargets
    public int hadcount(){
        int count = 0; 
        for (prediction: inferences){
            count += 1;
        }
        return count;
    }

    public static void collect_ready(){
        if (balls_present){
            //set intake command
        }

    }

    public boolean balls_present(){
        if(count > 10 && hasTarget){
            return true;
        }else{
            return false;
        }
    }
    // detect balls withing a certain areas of a camera -- look up hwo to narrow the camreas view 
}



