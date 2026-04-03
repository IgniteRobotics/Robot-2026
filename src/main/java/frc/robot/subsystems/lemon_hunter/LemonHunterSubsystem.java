package frc.robot.subsystems.lemon_hunter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LemonHunterSubsystem extends SubsystemBase{
    
    public LemonHunterSubsystem(){}

    @Override
    public void periodic(){
        var lemonHunterResults = LemonHunterConstants.lemonHunterCamera.getAllUnreadResults();
        for(var result : lemonHunterResults){
            
        }
    }
}
