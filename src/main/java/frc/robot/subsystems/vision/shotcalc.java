
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

public class VisionSystem extends SubsystemBase {
    //variables
    private final PhotonCamera photonCamera_Front;   
    private final PhotonPoseEstimation poseEstimator;
    private Optional<EstimatedRobotPose> latestEstrimatedPose = Optional.empty();
    private static final InterpoolingDoubleTreeMap TOFMap = new InterpoolingDoubleTreeMap();
    pirvate Vecotr<N3> coordinates;
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
      double hoodangle,
      double driveVelocity,
      double TOF, 
      double driveAngle

    ){}

    //pass mapping 
    private static final InterpolatingTreeMap<Double, Rotation2D> hoodthetmap =
     new InterpolatingTreeMap<>(InverseInterpolator.forDouble(),Rotation2d::interpolate);
    private static final InterpolatingTreeMap flywheelSpeedMap =
     new InterpolatingTreeMap();
    private static final InterpolatingTreeMap TOF =
     new InterpolatingTreeMap();
 
    //origin points
    /*Get hub  distance */

    static{
      minDist = 0.9;
      maxDist = 4.9;
      phaseDelay = 0.03;


      if()
    }


    }

