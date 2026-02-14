package frc.robot.subsystems.drive;

import frc.robot.preferences.DoublePreference;

public final class DrivePreferences {

  protected static DoublePreference translation_kP =
      new DoublePreference("Drive/Translation/kP", 0);
  protected static DoublePreference translation_kD =
      new DoublePreference("Drive/Translation/kD", 0);

  protected static DoublePreference rotation_kP = new DoublePreference("Drive/Rotation/kP", 0);
  protected static DoublePreference rotation_kD = new DoublePreference("Drive/Rotation/kD", 0);

  public static DoublePreference onemeter_speed =
      new DoublePreference("Drive/Speed for 1m (command)");
}
