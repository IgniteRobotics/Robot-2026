package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotorLeader;
  private final TalonFX indexerMotorFollower;
  private final TalonFX acceleratorMotor;

  @Logged(name = "Indexer Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity indexerVelocityTarget; // RotationsPerSecond

  @Logged(name = "Accelerator Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity acceleratorVelocityTarget;

  private VelocityVoltage indexerControl;
  private VelocityVoltage acceleratorControl;

  final SysIdRoutine m_sysIdRoutineIndexer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdIndexer_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setIndexerVoltage(output.magnitude()), null, this));

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

  public IndexerSubsystem() {
    indexerMotorLeader = new TalonFX(IndexerConstants.INDEXER_MOTOR_LEADER_ID);
    indexerMotorFollower = new TalonFX(IndexerConstants.INDEXER_MOTOR_FOLLOWER_ID);
    acceleratorMotor = new TalonFX(IndexerConstants.ACCELERATOR_MOTOR_ID);

    indexerMotorLeader.getConfigurator().apply(IndexerConstants.createLeaderMotorOutputConfigs());
    indexerMotorFollower
        .getConfigurator()
        .apply(IndexerConstants.createFollowerMotorOutputConfigs());

    indexerMotorLeader.getConfigurator().apply(IndexerConstants.createIndexerMotorSlot0Configs());
    indexerMotorFollower.getConfigurator().apply(IndexerConstants.createIndexerMotorSlot0Configs());

    acceleratorMotor.getConfigurator().apply(IndexerConstants.createAcceleratorMotorSlot0Configs());
    acceleratorMotor
        .getConfigurator()
        .apply(IndexerConstants.createAcceleratorMotorOutputsConfigs());

    indexerMotorFollower.setControl(
        new Follower(indexerMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    indexerVelocityTarget = RotationsPerSecond.of(0);
    acceleratorVelocityTarget = RotationsPerSecond.of(0);

    indexerControl = new VelocityVoltage(0);
    acceleratorControl = new VelocityVoltage(0);
  }

  @Override
  public void periodic() {
    // TODO:  removed for testing only.  PUT IT BACK!
    // indexerMotor.setControl(
    //     indexerControl.withVelocity(indexerVelocityTarget.in(RotationsPerSecond)));
  }

  private void setIndexerVoltage(double magnitude) {
    indexerMotorLeader.setVoltage(magnitude);
  }

  private void setAcceleratorVoltage(double magnitude) {
    acceleratorMotor.setVoltage(magnitude);
  }

  public Command startIndexerNoPID() {
    return run(() -> indexerMotorLeader.set(IndexerPreferences.indexerPercent.getValue()))
        .withName("Set Indexer Percent");
  }

  public Command startAcceleratorNoPID() {
    return run(() -> acceleratorMotor.set(IndexerPreferences.acceleratorPercent.getValue()))
        .withName("Set Acceleration Percent");
  }

  public Command stopIndexerNoPID() {
    return runOnce(() -> indexerMotorLeader.set(0)).withName("Stop Indexer Percent");
  }

  public Command stopAcceleratorNoPID() {
    return runOnce(() -> acceleratorMotor.set(0)).withName("Stop Accelerator Percent");
  }

  public Command startFullIndexingNoPID() {
    return run(() -> {
          indexerMotorLeader.set(IndexerPreferences.indexerPercent.getValue());
          acceleratorMotor.set(IndexerPreferences.acceleratorPercent.getValue());
        })
        .withName("Start Full Indexing No PID");
  }

  public Command stopFullIndexingNoPID() {
    return run(() -> {
          indexerMotorLeader.set(0);
          acceleratorMotor.set(0);
        })
        .withName("Stop Full Indexing No PID");
  }

  public Command indexCommand() {
    return runOnce(
            () ->
                indexerVelocityTarget =
                    RotationsPerSecond.of(IndexerPreferences.indexSpeed.getValue()))
        .withName("Index Lemons");
  }

  public Command stopCommand() {
    return runOnce(() -> indexerVelocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Indexing");
  }
}
