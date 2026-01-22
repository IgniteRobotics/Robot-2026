package frc.robot.subsystems.intake;

import frc.robot.preferences.DoublePreference;

public class IntakePreferences {
    public static DoublePreference rollerIntakeSpeed = new DoublePreference("Intake/Roller Intake Speed", 0.5);
    public static DoublePreference intakeCollectPosition = new DoublePreference("Intake/Stowed Intake Position", 3);
}
