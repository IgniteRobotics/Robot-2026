package frc.robot.subsystems.drive;

import frc.robot.preferences.DoublePreference;

public final class DrivePreferences {

  public static DoublePreference translation_kP =
      new DoublePreference("Drive/Translation/kP", DriveConstants.TRANSLATION_ALIGN_KP);
  public static DoublePreference translation_kD = new DoublePreference("Drive/Translation/kD", 0);

  public static DoublePreference rotation_kP =
      new DoublePreference("Drive/Rotation/kP", DriveConstants.ROTATION_ALIGN_KP);
  public static DoublePreference rotation_kD = new DoublePreference("Drive/Rotation/kD", 0);

  public static DoublePreference autoAim_kP = new DoublePreference("Drive/AutoAim/kP", 5);
  public static DoublePreference autoAim_kD = new DoublePreference("Drive/AutoAim/kD", 0);

  public static DoublePreference autoAimMaxSpeed =
      new DoublePreference("Drive/AutoAim/MaxDriveSpeed", 2);
}
