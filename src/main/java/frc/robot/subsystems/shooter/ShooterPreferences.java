package frc.robot.subsystems.shooter;

import frc.robot.preferences.DoublePreference;

public class ShooterPreferences {
  private ShooterPreferences() {}

  // TODO: Replace with formulas
  public static DoublePreference flywheelLaunchSpeed =
      new DoublePreference("Shooter/Launch Speed", 0.1); // in rotations per second
  public static DoublePreference hoodLaunchAngle =
      new DoublePreference("Shooter/Hood Launch Position", 3); // in rotations

  public static DoublePreference flywheelLaunchPercent =
      new DoublePreference("Shooter/Launch Percent (for without PID)", 0.1); // in percent

  public static DoublePreference hoodkP = new DoublePreference("Shooter/Hood/kP", 1);
  public static DoublePreference hoodkD = new DoublePreference("Shooter/Hood/kD", 0);

  public static DoublePreference hoodTargetPreference =
      new DoublePreference("Shooter/Hood/Target Rotations", 0);

  // unit is *radians per second** for velocity.
  public static DoublePreference tuningDefaultFlywheelRPS =
      new DoublePreference("Shooter/Tuning/FlywheelRPS", 1000.0 / 60.0 / 2 * Math.PI);
  public static DoublePreference tuningDefaultFlywheelStepRPS =
      new DoublePreference("Shooter/Tuning/FlywheelStepRPS", 100.0 / 60.0 / 2 * Math.PI);
  public static DoublePreference tuningDefaultHoodRotations =
      new DoublePreference("Shooter/Tuning/HoodRotations", 0.0);
  public static DoublePreference tuningDefaultHoodStepRotations =
      new DoublePreference("Shooter/Tuning/HoodStepRotations", 0.2);

  public static DoublePreference autoAimDeadline =
      new DoublePreference("Shooter/Auto Aim Deadline", 5.0); // in seconds
}
