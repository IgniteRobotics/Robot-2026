package frc.robot.subsystems.shooter;

import frc.robot.preferences.DoublePreference;

public class ShooterPreferences {
  private ShooterPreferences() {}

  // TODO: Replace with formulas
  public static DoublePreference flywheelLaunchSpeed =
      new DoublePreference("Shooter/Launch Speed", 0.1); // in rotations per second
  public static DoublePreference hoodLaunchAngle =
      new DoublePreference("Shooter/Hood Launch Position", 3); // in rotations
}
