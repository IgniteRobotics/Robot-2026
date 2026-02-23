package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import frc.robot.preferences.DoublePreference;

public final class DrivePreferences {

  protected static DoublePreference translation_kP =
      new DoublePreference("Drive/Translation/kP", DriveConstants.TRANSLATION_ALIGN_KP);
  protected static DoublePreference translation_kD =
      new DoublePreference("Drive/Translation/kD", 0);

  protected static DoublePreference rotation_kP =
      new DoublePreference("Drive/Rotation/kP", DriveConstants.ROTATION_ALIGN_KP);
  protected static DoublePreference rotation_kD = new DoublePreference("Drive/Rotation/kD", 0);

  public static DoublePreference onemeter_speed =
      new DoublePreference("Drive/Speed for 1m (command)");

  // ***** Auto Pilot / Drive to Pose ***** //
  public static DoublePreference autopilotMaxVelocity =
      new DoublePreference(
          "Drive/Autopilot/MaxVelocity", DriveConstants.AP_MAX_VELOCITY.in(MetersPerSecond));
  public static DoublePreference autopilotMaxAccel =
      new DoublePreference(
          "Drive/Autopilot/MaxAccel",
          DriveConstants.AP_MAX_ACCELERATION.in(MetersPerSecondPerSecond));

  public static DoublePreference autopilotMaxRotation =
      new DoublePreference(
          "Drive/Autopilot/MaxRotation",
          DriveConstants.AP_MAX_ANGULAR_VELOCITY.in(RotationsPerSecond));
  public static DoublePreference autopilotMaxAngularAccel =
      new DoublePreference(
          "Drive/Autopilot/MaxAngularAccel",
          DriveConstants.AP_MAX_ANGULAR_ACCELERATION.in(RotationsPerSecondPerSecond));

  public static DoublePreference autopilotLinearFFMinError =
      new DoublePreference(
          "Drive/Autopilot/LinearFFMinError",
          DriveConstants.AP_LINEAR_FF_MIN_ERROR.in(Centimeters));
  public static DoublePreference autopilotRotationFFMinError =
      new DoublePreference(
          "Drive/Autopilot/RotationFFMinError",
          DriveConstants.AP_ROTATION_FF_MIN_ERROR.in(Degrees));

  public static DoublePreference autopilotLinearFFMaxError =
      new DoublePreference(
          "Drive/Autopilot/LinearFFMaxError",
          DriveConstants.AP_LINEAR_FF_MAX_ERROR.in(Centimeters));
  public static DoublePreference autopilotRotationFFMaxError =
      new DoublePreference(
          "Drive/Autopilot/RotationFFMaxError",
          DriveConstants.AP_ROTATION_FF_MAX_ERROR.in(Degrees));

  public static DoublePreference autopilotMinAppliedVelocity =
      new DoublePreference(
          "Drive/Autopilot/MinAppliedVelocity",
          DriveConstants.AP_MIN_APPLIED_VELOCITY.in(MetersPerSecond));
}
