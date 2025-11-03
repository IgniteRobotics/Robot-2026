package frc.robot.statemachines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceState {

    private static AllianceState single_instance = null;

    private AllianceState() {
            
    }

    public static synchronized AllianceState getInstance()
    {
        if (single_instance == null)
            single_instance = new AllianceState();
        
        return single_instance;
    }

    public Alliance getAlliance(){
        return DriverStation.isDSAttached() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }
}