package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;

    private Angle climbTarget; // Rotations
    private PositionTorqueCurrentFOC climbControl;

    public ClimberSubsystem(){
        climbMotor = new TalonFX(ClimberConstants.CLIMB_MOTOR_ID);

        climbMotor.getConfigurator().apply(ClimberConstants.createClimbMotorSlot0Configs());
        climbMotor.getConfigurator().apply(ClimberConstants.createClimbSoftwareLimitSwitchConfigs());
        climbMotor.setPosition(0);
        climbTarget = Rotations.of(0);
        climbControl = new PositionTorqueCurrentFOC(0);
    }

    @Override
    public void periodic(){
        climbMotor.setControl(climbControl.withPosition(climbTarget));
    }

    private boolean atClimbSetpoint(){
        return Math.abs(climbMotor.getPosition().getValueAsDouble() - climbTarget.in(Rotations))
            < ClimberConstants.ALLOWABLE_CLIMB_ERROR;
    }

    public Command climbToPosition(Angle position) {
        return runOnce(() -> climbTarget = position)
            .andThen(Commands.waitUntil(() -> atClimbSetpoint()));
    }

    public Command resetClimb(){
        return climbToPosition(Rotations.of(0));
    }

    public Command climbToPreference(){
        return climbToPosition(Rotations.of(ClimberPreferences.pullPosition.getValue()));
    }

    public Command homeClimber(){
        return runEnd(
            () -> climbMotor.set(ClimberConstants.SAFE_HOMING_EFFORT),
            () -> climbMotor.setPosition(0))
        .until(
            () -> {
              return climbMotor.getStatorCurrent().getValueAsDouble()
                  > ClimberConstants.SAFE_STATOR_LIMIT;
            });
    }


}
