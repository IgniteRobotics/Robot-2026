package frc.robot.subsystems.intake;

import frc.robot.preferences.DoublePreference;

public class IntakePreferences {

  private IntakePreferences() {}
  ;

  public static DoublePreference rollerIntakeSpeed =
      new DoublePreference("Intake/Roller Intake Speed", 0.1); // in rotations per second
  public static DoublePreference intakeCollectPosition =
      new DoublePreference("Intake/Stowed Intake Position", 3); // in rotations
}
