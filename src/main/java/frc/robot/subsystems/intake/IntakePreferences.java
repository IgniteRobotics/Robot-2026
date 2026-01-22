package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.preferences.DoublePreference;

public class IntakePreferences {

    private IntakePreferences(){};

    public static DoublePreference rollerIntakeSpeed = new DoublePreference("Intake/Roller Intake Speed", 0.1);
    public static DoublePreference intakeCollectPosition = new DoublePreference("Intake/Stowed Intake Position", 3);
}
