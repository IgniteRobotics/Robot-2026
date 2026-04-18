package frc.robot.subsystems.pdh;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class PDHSubsystem extends SubsystemBase {

  private final PowerDistribution hub = new PowerDistribution(0, ModuleType.kRev);

  @Logged(name = "Bus Voltage", importance = Importance.CRITICAL)
  private Voltage busVoltage = Volts.of(0);

  @Logged(name = "Temperature (Celsius)", importance = Importance.CRITICAL)
  private Temperature temperature = Celsius.of(0);

  @Logged(name = "Total Current", importance = Importance.CRITICAL)
  private Current totalCurrent = Amps.of(0);

  @Logged(name = "Total Power", importance = Importance.CRITICAL)
  private Power totalPower = Watts.of(0);

  @Logged(name = "Total Energy", importance = Importance.CRITICAL)
  private Energy totalEnergy = Joules.of(0);

  public PDHSubsystem() {}

  @Override
  public void periodic() {
    busVoltage = Volts.of(hub.getVoltage());
    temperature = Celsius.of(hub.getTemperature());
    totalCurrent = Amps.of(hub.getTotalCurrent());
    totalPower = Watts.of(hub.getTotalPower());
    totalEnergy = Joules.of(hub.getTotalEnergy());

    reportToSmartDashboard();
  }

  public void reportToSmartDashboard() {
    SmartDashboard.putNumber("PDH/Bus Voltage", busVoltage.in(Volts));
    SmartDashboard.putNumber("PDH/Temperature (Celsius)", temperature.in(Celsius));
    SmartDashboard.putNumber("PDH/Total Current", totalCurrent.in(Amps));
    SmartDashboard.putNumber("PDH/Total Power", totalPower.in(Watts));
    SmartDashboard.putNumber("PDH/Total Energy", totalEnergy.in(Joules));
  }
}
