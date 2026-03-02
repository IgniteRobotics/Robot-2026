
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.DriveState;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class shotcalc extends SubsystemBase {
    //variables
    private final PhotonCamera photonCamera_Front;   
    private final PhotonPoseEstimation poseEstimator;
    private Optional<EstimatedRobotPose> latestEstrimatedPose = Optional.empty();
    private static final InterpoolingDoubleTreeMap TOFMap = new InterpoolingDoubleTreeMap();
    private Vecotr<N3> coordinates;
    List<PhotonTrackedTarget> targets = result.getTargets();



    public VisionMeasuments(EstimatedRobotPose pose, double time, Vector<N3> trust){
      estimatedPose = pose;
      timestamp = time;
      trustValues = trust;
    }

    public EastimatedRobotPose getEstimatedPose(){
        return estimatedPose;

    }
    public double getTimestamp() {
      return timestamp;
    }

    public Vector<N3> getTrust() {
      return trustValues;
    }
    //data
    public record LauncingConditions(
      boolean isValid,
      double distance, 
      double flywheelspeed,
      double idelflywheelspeed,
      boolean passing
      double hoodangle,
      double driveVelocity,
      double TOF, 
      double driveAngle,
      int tagnum

    ){}
    private LauncingConditions latestParam = null
    //shot mapping 
    private static final InterpolatingTreeMap<Double, Rotation2D> hoodthetmap =
     new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),Rotation2d::interpolate);
    private static final InterpolatingTreeMap flywheelSpeedMap =
     new InterpolatingTreeMap();
    private static final InterpolatingDoubleTreeMap TOF =
     new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap  passingFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
 
    //origin points
    /*Get hub  distance */

    static{
      minDist = 0.9;
      maxDist = 4.9;
      phaseDelay = 0.03;

      hoodthetmap.put()
      hoodthetmap.put()
      hoodthetmap.put()
      hoodthetmap.put()
      hoodthetmap.put()

      TOF.put();
      TOF.put();
      TOF.put();
      TOF.put();
      TOF.put();
      TOF.put();


      flywheelSpeedMap.put();
      flywheelSpeedMap.put();
      flywheelSpeedMap.put();
      flywheelSpeedMap.put();
      flywheelSpeedMap.put();
      flywheelSpeedMap.put();

      
    }
    //preset values
    public static final double hubPresentDistance;
    public static final double LimboPresentDistance;
    public static final double ClimbPresentDistance;



    //passed tags - refine this so we get one absolute tag and use te documentation from photonvision to get relative poser -> robot moves to compatable shooting pose 
    var result = photonCamera_Front.getLatestResult();
    if(result.hasTargets()){
      int TagID = results.getBestTarget().getFiducialId(); // got off of photon documentation 
    }

    public int getTagID(){
      return TagID; // check if it return the latest tag if possible may need ot be adjested to multitag for abs position
    }

    //pose for shot and run 
    Pose2d estimatedPose = RobotContainer.getInstance().getEstimatedPose(); // idk if this is the correc dir for instnace
    // get robot velocity - ask where the data is coming though for that
    
    Translation2d target = passing ?getTagID
    // Pose2d -- lancher position relative to robot



    //things to ask launcher relative to center of robot


    // have settign and poolena conditions for when we launch vs when we are shooting still 


    //updated parameter to put into robot for shooting 


    //get the ange we are at when we are moving 



    }

