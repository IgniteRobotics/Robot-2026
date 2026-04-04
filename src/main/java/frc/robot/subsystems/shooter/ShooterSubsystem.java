package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.statemachines.LaunchState;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotorLeftLeader;
  private final TalonFX flywheelMotorRight;
  private final TalonFX flywheelMotorLeftFollower;
  private final TalonFX hoodMotor;

  @Logged(name = "Velocity Target rads/s", importance = Importance.CRITICAL)
  private AngularVelocity velocityTarget; // *rads* Per Second is the base unit.

  private VelocityVoltage velocityControl;

  private final LaunchState launchState = LaunchState.getInstance();

  @Logged(name = "Hood Target (radians)", importance = Importance.CRITICAL)
  private Angle hoodTarget; // radians is the base unit.

  private PositionVoltage hoodControl;

  final SysIdRoutine m_sysIdRoutineFlywheel =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Second).of(0.2), // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setFlywheelVoltage(output.magnitude()), null, this));

  public ShooterSubsystem() {
    flywheelMotorLeftLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEFT_LEADER_MOTOR_ID);
    flywheelMotorLeftFollower = new TalonFX(ShooterConstants.FLYWHEEL_LEFT_FOLLOWER_MOTOR_ID);
    flywheelMotorRight = new TalonFX(ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    flywheelMotorLeftLeader
        .getConfigurator()
        .apply(ShooterConstants.createLeftFlywheelLeaderMotorOutputConfigs());
    flywheelMotorLeftFollower
        .getConfigurator()
        .apply(ShooterConstants.createLeftFlywheelFollowerMotorOutputConfigs());
    flywheelMotorRight
        .getConfigurator()
        .apply(ShooterConstants.createRightFlywheelMotorOutputConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorOutputConfigs());

    flywheelMotorLeftLeader
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelCurrentLimitsConfigs());
    flywheelMotorLeftFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelCurrentLimitsConfigs());
    flywheelMotorRight
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelCurrentLimitsConfigs());

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodCurrentLimitsConfigs());

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotionMagicConfigs());

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());

    flywheelMotorLeftFollower.setControl(new Follower(5, MotorAlignmentValue.Aligned));
    flywheelMotorRight.setControl(new Follower(5, MotorAlignmentValue.Opposed));

    hoodTarget = Rotations.of(0);
    hoodControl = new PositionVoltage(hoodTarget);

    velocityTarget = RotationsPerSecond.of(0);
    velocityControl = new VelocityVoltage(velocityTarget);

    hoodMotor.setPosition(0);
  }

  private void setFlywheelVoltage(double magnitude) {
    flywheelMotorLeftLeader.setVoltage(magnitude);
    flywheelMotorLeftFollower.setVoltage(magnitude);
    flywheelMotorRight.setVoltage(magnitude);
  }

  @Override
  public void periodic() {
    launchState.refreshRequest();
    flywheelMotorLeftLeader.setControl(
        velocityControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    hoodMotor.setControl(hoodControl.withPosition(hoodTarget));
  }

  public void setFlywheelMotorOutput(double output) {
    flywheelMotorLeftLeader.set(output);
    flywheelMotorLeftFollower.set(output);
    flywheelMotorRight.set(output);
  }

  @Logged(name = "Hood Angle Rotations", importance = Importance.CRITICAL)
  public double getHoodTargetRotations() {
    return hoodTarget.in(Rotations);
  }

  @Logged(name = "At Hood Setpoint", importance = Importance.CRITICAL)
  public boolean atHoodSetpoint() {
    return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodTarget.in(Rotations))
        < ShooterConstants.ALLOWABLE_HOOD_ERROR;
  }

  @Logged(name = "Velocity Target RPM", importance = Importance.CRITICAL)
  public double getFlywheelTargetRPM() {
    return velocityTarget.in(RotationsPerSecond) * 60;
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

  public Command stopFlywheelCommand() {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Spinning Flywheel");
  }

  public Command stowHood() {
    return runOnce(() -> hoodTarget = Rotations.of(0));
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
    return runOnce(
            () -> {
              velocityTarget =
                  RotationsPerSecond.of(ShooterPreferences.tuningDefaultFlywheelRPS.getValue());
              hoodTarget = Rotations.of(ShooterPreferences.tuningDefaultHoodRotations.getValue());
            })
        .withName("Start Shooter Tuning");
  }

  public Command stopShooterTuningCommand() {
    return stopFlywheelCommand().andThen(stowHood()).withName("Stop Shooter Tuning");
  }

  public Command spinFlywheelPostCommand() {
    return runOnce(
            () -> {
              velocityTarget = RotationsPerSecond.of(74.5);
              hoodTarget = Rotations.of(4);
            })
        .withName("Start Spinning Flywheel");
  }

  public Command spinFlywheelRanged() {
    return run(
        () -> {
          velocityTarget = launchState.getLaunchRequest().getFlywheelVelocity();
          hoodTarget = launchState.getLaunchRequest().getHoodTarget();
        });
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineFlywheel.dynamic(direction);
  }
}
