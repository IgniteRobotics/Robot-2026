package frc.robot.subsystems.intake;

import edu.wpi.first.units.*;
import frc.robot.preferences.DoublePreference;

public class IntakePreferences {

  private IntakePreferences() {}
  ;

  public static DoublePreference rollerIntakeSpeed =
      new DoublePreference("Intake/Roller Intake Speed", 0.1); // in rotations per second
  public static DoublePreference intakeCollectPosition =
      new DoublePreference("Intake/Stowed Intake Position", 3); // in rotations

  public static DoublePreference extendPercent =
      new DoublePreference("Intake/Extend Percent (for without PID)", 0.1); // in percent
  public static DoublePreference retractPercent =
      new DoublePreference("Intake/Retract Percent (for without PID)", -0.1); // in percent

  public static DoublePreference rollerIntakePercent =
      new DoublePreference("Intake/Roller Intake Percent (for without PID)", 0.1); // in percent
  public static DoublePreference rollerOuttakePercent =
      new DoublePreference("Intake/Roller Outtake Percent (for without PID)", -0.1); // in percent
  public static DoublePreference testRollerIntakePercent =
      new DoublePreference("Intake/Roller Test Percent (for without PID)", 0.05); // in percent

  public static DoublePreference dislodgePosition =
      new DoublePreference("Intake/Extension Dislodge Position", 10.0);

  public static DoublePreference noPIDWait =
      new DoublePreference("Intake/Extension Deadline (No PID)", 1.0); // in seconds

  public static DoublePreference outpostReloadWait =
      new DoublePreference("Intake/Outpost Reload Waittime", 2.5); // in seconds

  public static DoublePreference resistanceCurrentLimit =
      new DoublePreference(
          "Intake/Compliant Resistance Current Limit",
          IntakeConstants.COMPLIANT_RESISTANCE_CURRENT_LIMIT.in(Units.Amps));
}
