package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.IntakeConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotorLeader;
  private final TalonFX flywheelMotorFollower;
  private final TalonFX hoodMotor;

  private final LaunchRequestBuilder launchRequestBuilder;

  @Logged(name = "Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity velocityTarget; // Rotations Per Second

  private VelocityVoltage velocityControl;

  @Logged(name = "Hood Target", importance = Importance.CRITICAL)
  private Angle hoodTarget; // Rotations

  private PositionTorqueCurrentFOC hoodControl;

  final SysIdRoutine m_sysIdRoutineFlywheel =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setFlywheelVoltage(output.magnitude()), null, this));

  public class LaunchRequest {
    private Angle launchHoodTarget;
    private AngularVelocity launchVelocity;

    public LaunchRequest(Angle theta, LinearVelocity velocity) {
      // hood angle in degrees (0 degrees is all the way back) is converted to motor rotations
      launchHoodTarget =
          Rotations.of(
              theta.in(Degrees) * ShooterConstants.ROTATIONS_PER_LAUNCH_DEGREE.in(Rotations));
      // meters per second is converted to rotations per second of the flywheel
      launchVelocity =
          RotationsPerSecond.of(
              velocity.in(MetersPerSecond)
                  / (2 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS.in(Meters)));
    }

    public Angle getHoodTarget() {
      return launchHoodTarget;
    }

    public AngularVelocity getVelocityTarget() {
      return launchVelocity;
    }
  }

  public ShooterSubsystem() {
    flywheelMotorLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID);
    flywheelMotorFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createLeaderMotorOutputConfigs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFollowerMotorOutputConfigs());

    velocityTarget = RotationsPerSecond.of(0);
    velocityControl = new VelocityVoltage(0);

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorOutputConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
    hoodTarget = Rotations.of(0);
    hoodControl = new PositionTorqueCurrentFOC(0);

    launchRequestBuilder = new MappedLauchRequestBuilder(() -> ShooterConstants.BLUE_TARGET);
  }

  @Override
  public void periodic() {
    flywheelMotorLeader.setControl(
        velocityControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    hoodMotor.setControl(hoodControl.withVelocity(hoodTarget.in(Rotations)));
    launchRequestBuilder.createLaunchRequest();
  }

  public Command spinFlywheelCommand() {
    return runOnce(
            () ->
                velocityTarget =
                    RotationsPerSecond.of(ShooterPreferences.flywheelLaunchSpeed.getValue()))
        .withName("Start Spinning Flywheel");
  }

  public Command stopFlywheelCommand() {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Spinning Flywheel");
  }

  private void setFlywheelVoltage(double magnitude) {
    flywheelMotorLeader.setVoltage(magnitude);
  }

  @Logged(name = "At Hood Setpoint", importance = Importance.CRITICAL)
  public boolean atHoodSetpoint() {
    return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodTarget.in(Rotations))
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setHoodCommand(Angle position) {
    return runOnce(() -> hoodTarget = position)
        .andThen(Commands.waitUntil(() -> atHoodSetpoint()))
        .withName("Set Hood Angle");
  }

  public Command launchLemonsCommand() {
    return setHoodCommand(Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue()))
        .andThen(spinFlywheelCommand())
        .withName("Start Launching Lemons");
  }

  public Command launchLemonsCommandNoPID() {
    return runOnce(
            () -> {
              flywheelMotorLeader.set(ShooterPreferences.flywheelLaunchPercent.getValue());
              flywheelMotorFollower.set(ShooterPreferences.flywheelLaunchPercent.getValue());
            })
        .withName("Start Launching Lemons (No PID)");
  }

  public Command stopLaunchLemonsNoPIDCommand() {
    return runOnce(
            () -> {
              flywheelMotorLeader.set(0);
              flywheelMotorFollower.set(0);
            })
        .withName("Stop Launching Lemons (No PID)");
  }

  public Command stowCommand() {
    return stopFlywheelCommand().andThen(setHoodCommand(Rotations.of(0))).withName("Stow Shooter");
  }

  public Command homeShooterCommand() {
    return runEnd(
            () -> hoodMotor.set(ShooterConstants.SAFE_HOMING_EFFORT.Output),
            () -> hoodMotor.setPosition(0))
        .until(
            () -> {
              return hoodMotor.getStatorCurrent().getValueAsDouble()
                  > ShooterConstants.SAFE_STATOR_LIMIT.in(Amp);
            });
  }
}
