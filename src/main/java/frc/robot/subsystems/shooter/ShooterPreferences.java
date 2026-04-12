package frc.robot.subsystems.shooter;

import frc.robot.preferences.DoublePreference;

public class ShooterPreferences {
  private ShooterPreferences() {}

  public static DoublePreference flywheelLaunchSpeed =
      new DoublePreference("Shooter/Launch Speed", 0.1); // in rotations per second

  public static DoublePreference hoodLaunchAngle =
      new DoublePreference("Shooter/Hood Launch Position", 3); // in rotations

  // unit is *radians per second** for velocity.
  public static DoublePreference tuningDefaultFlywheelRPS =
      new DoublePreference("Shooter/Tuning/FlywheelRPS", 1000.0 / 60.0 / 2 * Math.PI);
  public static DoublePreference tuningDefaultFlywheelStepRPS =
      new DoublePreference("Shooter/Tuning/FlywheelStepRPS", 100.0 / 60.0 / 2 * Math.PI);
  public static DoublePreference tuningDefaultHoodRotations =
      new DoublePreference("Shooter/Tuning/HoodRotations", 0.0);
  public static DoublePreference tuningDefaultHoodStepRotations =
      new DoublePreference("Shooter/Tuning/HoodStepRotations", 0.2);

  public static DoublePreference passingFlywheelSpeed =
      new DoublePreference("Shooter/Tuning/Flywheel Pass Speed", 75);
}
