package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.preferences.DoublePreference;
import frc.robot.statemachines.LaunchState;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotorLeader;
  private final TalonFX flywheelMotorFollower;
  private final TalonFX hoodMotor;

  @Logged(name = "Velocity Target rads/s", importance = Importance.CRITICAL)
  private AngularVelocity velocityTarget; // *rads* Per Second is the base unit.

  private VelocityVoltage velocityControl;

  @Logged(name = "Hood Target (radians)", importance = Importance.CRITICAL)
  private Angle hoodTarget; // radians is the base unit.

  private PositionVoltage hoodControl;

  private final LaunchState launchState = LaunchState.getInstance();

  final SysIdRoutine m_sysIdRoutineFlywheel =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setFlywheelVoltage(output.magnitude()), null, this));

  public ShooterSubsystem() {
    flywheelMotorLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID);
    flywheelMotorFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createLeaderMotorOutputConfigs());
    flywheelMotorLeader
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelCurrentLimitsConfigs());

    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFollowerMotorOutputConfigs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelCurrentLimitsConfigs());

    velocityTarget = RotationsPerSecond.of(0);
    velocityControl = new VelocityVoltage(0);

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorOutputConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodCurrentLimitsConfigs());
    hoodTarget = Rotations.of(0);
    hoodControl = new PositionVoltage(0);
  }

  @Override
  public void periodic() {
    launchState.refreshRequest();
    flywheelMotorLeader.setControl(
        velocityControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    flywheelMotorFollower.setControl(
        velocityControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    hoodMotor.setControl(hoodControl.withPosition(hoodTarget));
  }

  @Logged(name = "Velocity Target RPM", importance = Importance.CRITICAL)
  public double getFlywheelTargetRPM() {
    return velocityTarget.in(RotationsPerSecond) * 60;
  }

  @Logged(name = "Hood Angle Rotations", importance = Importance.CRITICAL)
  public double getHoodTargetRotations() {
    return hoodTarget.in(Rotations);
  }

  public Command spinFlywheelCommand() {
    return runOnce(
            () -> {
              velocityTarget =
                  RotationsPerSecond.of(ShooterPreferences.flywheelLaunchSpeed.getValue());
              hoodTarget = Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue());
            })
        .withName("Start Spinning Flywheel");
  }

  public Command spinFlywheelPostCommand() {
    return runOnce(
            () -> {
              velocityTarget = RotationsPerSecond.of(74.5);
              hoodTarget = Rotations.of(4);
            })
        .withName("Start Spinning Flywheel");
  }

  public Command spinFlywheelCommand(AngularVelocity v) {
    return runOnce(() -> velocityTarget = v).withName("Spinning To AV");
  }

  public Command spinFlyWheelCommand(DoublePreference d) {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(d.getValue()));
  }

  public Command stopFlywheelCommand() {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Spinning Flywheel");
  }

  public Command stowHood() {
    return runOnce(() -> hoodTarget = Rotations.of(0));
  }

  public Command setHoodToPreference() {
    return runOnce(
        () -> hoodTarget = Rotations.of(ShooterPreferences.hoodTargetPreference.getValue()));
  }

  private void setFlywheelVoltage(double magnitude) {
    flywheelMotorLeader.setVoltage(magnitude);
    flywheelMotorFollower.setVoltage(magnitude);
  }

  @Logged(name = "At Hood Setpoint", importance = Importance.CRITICAL)
  public boolean atHoodSetpoint() {
    return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodTarget.in(Rotations))
        < ShooterConstants.ALLOWABLE_HOOD_ERROR;
  }

  public Command setHoodCommand(Angle position) {
    return runOnce(() -> hoodTarget = position)
        .andThen(Commands.waitUntil(() -> atHoodSetpoint()))
        .withName("Set Hood Angle");
  }

  public Command spinFlywheelRanged() {
    return run(
        () -> {
          velocityTarget = launchState.getLaunchRequest().getFlywheelVelocity();
          hoodTarget = launchState.getLaunchRequest().getHoodTarget();
        });
  }

  public Command spinFlywheelHardCoded() {
    return run(
        () -> {
          velocityTarget = RotationsPerSecond.of(66.5);
          hoodTarget = Rotations.of(2.38);
        });
  }

  public Command launchLemonsCommandNoPID() {
    return setHoodCommand(Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue()))
        .andThen(
            runOnce(
                () -> {
                  flywheelMotorLeader.set(ShooterPreferences.flywheelLaunchPercent.getValue());
                  flywheelMotorFollower.set(ShooterPreferences.flywheelLaunchPercent.getValue());
                }))
        .withName("Start Launching Lemons (No PID)");
  }

  public Command stopLaunchLemonsNoPIDCommand() {
    return setHoodCommand(Rotations.of(0))
        .andThen(
            runOnce(
                () -> {
                  flywheelMotorLeader.set(0);
                  flywheelMotorFollower.set(0);
                }))
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

  public Command increaseFlywheelCommand() {
    return runOnce(
            () ->
                velocityTarget =
                    RotationsPerSecond.of(
                        velocityTarget.in(RotationsPerSecond)
                            + ShooterPreferences.tuningDefaultFlywheelStepRPS.getValue()))
        .withName("Increase FlyWheel Speed");
  }

  public Command decreaseFlywheelCommand() {
    return runOnce(
            () ->
                velocityTarget =
                    RotationsPerSecond.of(
                        velocityTarget.in(RotationsPerSecond)
                            - ShooterPreferences.tuningDefaultFlywheelStepRPS.getValue()))
        .withName("Decrease FlyWheel Speed");
  }

  public Command increaseHoodCommand() {
    return runOnce(
            () ->
                hoodTarget =
                    Rotations.of(
                        hoodTarget.in(Rotations)
                            + ShooterPreferences.tuningDefaultHoodStepRotations.getValue()))
        .withName("Increase Hood Angle");
  }

  public Command decreaseHoodCommand() {
    return runOnce(
            () ->
                hoodTarget =
                    Rotations.of(
                        hoodTarget.in(Rotations)
                            - ShooterPreferences.tuningDefaultHoodStepRotations.getValue()))
        .withName("Decrease Hood Angle");
  }

  public Command startShooterTuningCommand() {
    return spinFlyWheelCommand(ShooterPreferences.tuningDefaultFlywheelRPS)
        .andThen(
            setHoodCommand(Rotations.of(ShooterPreferences.tuningDefaultHoodRotations.getValue())))
        .withName("Start Shooter Tuning");
  }

  public Command stopShooterTuningCommand() {
    return stopFlywheelCommand()
        .andThen(
            setHoodCommand(Rotations.of(ShooterPreferences.tuningDefaultHoodRotations.getValue())))
        .withName("Stop Shooter Tuning");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.dynamic(direction);
  }
}
