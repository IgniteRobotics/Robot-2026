package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotor;

  @Logged(name = "Indexer Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity indexerVelocityTarget; // RotationsPerSecond

  private VelocityVoltage indexerControl;

  public IndexerSubsystem() {
    indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID);
    indexerMotor.getConfigurator().apply(IndexerConstants.createIndexerMotorSlot0Configs());
    indexerVelocityTarget = RotationsPerSecond.of(0);
    indexerControl = new VelocityVoltage(0);
  }

  @Override
  public void periodic() {
    indexerMotor.setControl(
        indexerControl.withVelocity(indexerVelocityTarget.in(RotationsPerSecond)));
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
