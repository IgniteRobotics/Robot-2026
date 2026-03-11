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
import frc.robot.statemachines.LaunchState;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotorLeader;
  private final TalonFX flywheelMotorFollower;
  private final TalonFX hoodMotor;
  private final TalonFX acceleratorMotor;

  @Logged(name = "Velocity Target rads/s", importance = Importance.CRITICAL)
  private AngularVelocity velocityTarget; // *rads* Per Second is the base unit.

  @Logged(name = "Hood Target (radians)", importance = Importance.CRITICAL)
  private Angle hoodTarget; // radians is the base unit.

  private VelocityVoltage velocityControl;
  private PositionVoltage hoodControl;
  private VelocityVoltage acceleratorControl;

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

  final SysIdRoutine m_sysIdRoutineAccelerator =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdAccelerator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setAcceleratorVoltage(output.magnitude()), null, this));

  public ShooterSubsystem() {
    flywheelMotorLeader = new TalonFX(ShooterConstants.FLYWHEEL_LEADER_MOTOR_ID);
    flywheelMotorFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
    acceleratorMotor = new TalonFX(ShooterConstants.ACCELERATOR_MOTOR_ID);

    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorLeader.getConfigurator().apply(ShooterConstants.createLeaderMotorOutputConfigs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    flywheelMotorFollower
        .getConfigurator()
        .apply(ShooterConstants.createFollowerMotorOutputConfigs());
    // flywheelMotorFollower.setControl(
    //    new Follower(flywheelMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    acceleratorMotor.getConfigurator().apply(ShooterConstants.createAcceleratorMotorSlot0Configs());
    acceleratorMotor
        .getConfigurator()
        .apply(ShooterConstants.createAcceleratorMotorOutputsConfigs());

    acceleratorControl = new VelocityVoltage(0);

    velocityTarget = RotationsPerSecond.of(0);
    velocityControl = new VelocityVoltage(0);

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorOutputConfigs());
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
    acceleratorMotor.setControl(
        acceleratorControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    
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

  /* Main Commands */
  public Command launchLemonsCommand() {
    return setHoodCommand(Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue()))
        .andThen(spinFlywheelCommand())
        .withName("Start Launching Lemons");
  }

  public Command launchLemonsRanged() { // Main command for start launching
    return spinFlywheelRanged();
  }

  public Command stopLaunchingLemons() { // Main command for stopping - stop flywheel and stow hood
    return stopFlywheelCommand().andThen(stowHood());
  }

  /* Flywheel Commands */
  public Command spinFlywheelCommand() {
    return runOnce(
            () -> {
              velocityTarget =
                  RotationsPerSecond.of(ShooterPreferences.flywheelLaunchSpeed.getValue());
              hoodTarget = Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue());
            })
        .withName("Start Spinning Flywheel");
  }

  public Command spinFlywheelCommand(AngularVelocity v) {
    return runOnce(() -> velocityTarget = v).withName("Spinning To AV");
  }

  public Command stopFlywheelCommand() {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Spinning Flywheel");
  }

  private void setFlywheelVoltage(double magnitude) {
    flywheelMotorLeader.setVoltage(magnitude);
    flywheelMotorFollower.setVoltage(magnitude);
  }

  public Command spinFlywheelRanged() {
    return run(
        () -> {
          velocityTarget = launchState.getLaunchRequest().getFlywheelVelocity();
          hoodTarget = launchState.getLaunchRequest().getHoodTarget();
        });
  }

  public Command spinFlywheelHardCoded() {
    return run(() -> {
          velocityTarget = RotationsPerSecond.of(66.5);
          hoodTarget = Rotations.of(2.38);
        })
        .andThen(startAcceleratorNoPID());
  }

  /* Hood Commands */
  public Command stowHood() {
    return runOnce(() -> hoodTarget = Rotations.of(0));
  }

  public Command setHoodToPreference() {
    return runOnce(
        () -> hoodTarget = Rotations.of(ShooterPreferences.hoodTargetPreference.getValue()));
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

  /* Accelerator Commands */
  private void setAcceleratorVoltage(double magnitude) {
    acceleratorMotor.setVoltage(magnitude);
  }

  /* NoPID Commands */
  public Command startAcceleratorNoPID() {
    return run(() -> acceleratorMotor.set(ShooterPreferences.acceleratorPercent.getValue()))
        .withName("Set Acceleration Percent");
  }

  public Command stopAcceleratorNoPID() {
    return runOnce(() -> acceleratorMotor.set(0)).withName("Stop Accelerator Percent");
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

  /* Testing Commands */
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
                    RadiansPerSecond.of(
                        velocityTarget.magnitude()
                            + ShooterPreferences.tuningDefaultFlywheelStepRPS.getValue()))
        .withName("Increase FlyWheel Speed");
  }

  public Command decreaseFlywheelCommand() {
    return runOnce(
            () ->
                velocityTarget =
                    RadiansPerSecond.of(
                        velocityTarget.magnitude()
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
    return spinFlywheelCommand(
            RadiansPerSecond.of(ShooterPreferences.tuningDefaultFlywheelRPS.getValue()))
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
