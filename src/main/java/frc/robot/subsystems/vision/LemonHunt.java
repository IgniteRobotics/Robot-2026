import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class LemonHunt {
    PhotonPipelineResult result = photonCamera_AI.getLatestResult();
    List<PhotonTrackedTarget> inferences = result.getTargets();
    @Logged (name = "AI inferences Positions")
    List<Translation3d> pos =  new ArrayList<>(); // idk if i should change to poe3d or translation

    @Logged(name = "nearest Targets")
    List<Pose3d> nearest_lemons = new ArrayList();

    @Logged (name = "AI inferences")
    List<PhotonPipelineResult> infer = photonCamera_AI.getAllUnreadResults();

    private boolean hasTargets = result.hasTargets();
    public class LemonHunt(){
        

    }

    

    public void nearest_kluster(){
        //Short break to learn about this 
    }

    public Translation3d nearest(){
        double minDist = Double.MAX_VALUE;
        for(PhotonTrackedTarget lemons: pos){
            Translation3d xyz = target.getBestCameraToTarget().getTranslation();
            double dist = xyz.getNorm();

            if(dist < minDist){
                minDist = dist;
                nearest = xyz;
            }

            }
        return nearest;
        }
    public List<Pose3d> getNearestLemonBasket(){
        int lemon_count = 0; 
        for(Pose3d lemons: pos){
            if((lemons.getX < (nearest.getX + 1)) && (lemons.getZ < (nearest.getZ + 2))){
                int ID = lemons.getDetectedObjectClassID();
                nearest_lemons.add(ID, (lemons));
                lemon_count += 1;
            }
            
        }
        return nearest_lemons;

    }
    public Pose3d getLemonDistances(Pose2d robotPose, PhotonTrackedTarget target, double cameraHeight, double objectHeight, double cameraPitch){
            double targetPitch = Math.toRadians(target.getPitch());
            double targetYaw = Math.toRadians(target.getYaw());

            double distance = PhotonUtils.calculateDistanceToTarget(
                cameraHeight,
                objectHeight,
                cameraPitch,
                targetPitch
            );
            double dx = distance * Math.cos(targetYaw);
            double dy = distance * Math.sin(targetYaw);

        return new Pose3d(
            robotPose.getX() + dx,
            robotPose.getY() + dy,
            objectHeight,
            new Rotation3d(0, 0, targetYaw)
        );

    }

    }



