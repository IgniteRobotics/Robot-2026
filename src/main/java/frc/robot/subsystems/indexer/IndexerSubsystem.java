package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFX acceleratorMotor;

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
    indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_LEADER_ID);
    acceleratorMotor = new TalonFX(IndexerConstants.ACCELERATOR_MOTOR_ID);

    indexerMotor.getConfigurator().apply(IndexerConstants.createIndexerMotorOutputConfigs());
    acceleratorMotor
        .getConfigurator()
        .apply(IndexerConstants.createAcceleratorMotorOutputsConfigs());

    indexerMotor.getConfigurator().apply(IndexerConstants.createIndexerCurrentLimitsConfigs());
    acceleratorMotor
        .getConfigurator()
        .apply(IndexerConstants.createAcceleratorCurrentLimitsConfigs());
  }

  // SysID Helpers
  private void setIndexerVoltage(double magnitude) {
    indexerMotor.setVoltage(magnitude);
  }

  private void setAcceleratorVoltage(double magnitude) {
    acceleratorMotor.setVoltage(magnitude);
  }

  public Command startFullIndexingNoPID() {
    return runEnd(
        () -> {
          indexerMotor.set(IndexerConstants.INDEXER_POWER);
          acceleratorMotor.set(IndexerConstants.ACCELERATOR_POWER);
        },
        () -> {
          indexerMotor.set(0);
          acceleratorMotor.set(0);
        });
  }

  public Command stopFullIndexingNoPID() {
    return runOnce(
            () -> {
              indexerMotor.set(0);
              acceleratorMotor.set(0);
            })
        .withName("Stop Full Indexing No PID");
  }

  public Command startIndexerReverseNoPID() {
    return runOnce(() -> indexerMotor.set(-1 * IndexerConstants.INDEXER_POWER))
        .withName("Set Indexer Reverse Percent");
  }

  public Command stopIndexerNoPID() {
    return runOnce(() -> indexerMotor.set(0)).withName("Stop Indexer Percent");
  }

  public Command testCommand() {
    return startFullIndexingNoPID()
        .andThen(new WaitCommand(2))
        .andThen(stopFullIndexingNoPID())
        .andThen(startIndexerReverseNoPID())
        .andThen(new WaitCommand(1))
        .andThen(stopIndexerNoPID())
        .withName("Indexer Test Command");
  }
}
