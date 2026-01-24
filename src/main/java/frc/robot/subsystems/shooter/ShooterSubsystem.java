package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private final TalonFX flywheelMotor;
    private final TalonFX hoodMotor;

    private AngularVelocity launchVelocityTarget; //Rotations Per Second
    private VelocityVoltage launchControl;

    private Angle hoodTarget; //Rotations
    private PositionTorqueCurrentFOC hoodControl;


    public ShooterSubsystem(){
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

        flywheelMotor.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
        launchVelocityTarget = RotationsPerSecond.of(0);
        launchControl = new VelocityVoltage(0);

        hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
        hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
        hoodTarget = Rotations.of(0);
        hoodControl = new PositionTorqueCurrentFOC(0);
    }

    @Override
    public void periodic(){}
    

    
}
