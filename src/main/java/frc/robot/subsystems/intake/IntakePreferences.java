package frc.robot.subsystems.intake;

import edu.wpi.first.units.*;
import frc.robot.preferences.DoublePreference;

public class IntakePreferences {

  private IntakePreferences() {}

  public static DoublePreference rollerIntakePercent =
      new DoublePreference("Intake/Roller Intake Percent (for without PID)", 0.80); // in percent
  public static DoublePreference rollerOuttakePercent =
      new DoublePreference("Intake/Roller Outtake Percent (for without PID)", -0.1); // in percent
  public static DoublePreference testRollerIntakePercent =
      new DoublePreference("Intake/Roller Test Percent (for without PID)", 0.05); // in percent

  public static DoublePreference dislodgePosition =
      new DoublePreference("Intake/Extension Dislodge Position", 10.0);

  public static DoublePreference outpostReloadWait =
      new DoublePreference("Intake/Outpost Reload Waittime", 2.5); // in seconds

  public static DoublePreference resistanceCurrentLimit =
      new DoublePreference(
          "Intake/Compliant Resistance Current Limit",
          IntakeConstants.COMPLIANT_RESISTANCE_CURRENT_LIMIT.in(Units.Amps));

  public static DoublePreference agitatePosition1 =
      new DoublePreference("Intake/Agitate/Position 1", 10.0);

  public static DoublePreference agitatePosition2 =
      new DoublePreference("Intake/Agitate/Position 2", 6.0);

  public static DoublePreference springykP =
      new DoublePreference("Intake/Springy/kP", IntakeConstants.EXTENSION_SPRINGY_KP);
  public static DoublePreference springykD =
      new DoublePreference("Intake/Springy/kD", IntakeConstants.EXTENSION_SPRINGY_KD);
}
