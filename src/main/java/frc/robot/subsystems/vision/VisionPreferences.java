package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.generated.TunerConstants;
import frc.robot.preferences.DoublePreference;

final class VisionPreferences {

  private VisionPreferences() {}

  protected static DoublePreference xyStdDevCoef = new DoublePreference("Vision/xyStdDevCoef", VisionConstants.XY_STD_DEV_COEFFICIENT);
  protected static DoublePreference thetaStdDevCoef = new DoublePreference("Vision/thetaStdDevCoef", VisionConstants.THETA_STD_DEV_COEFFICIENT);
  protected static DoublePreference jumpLimit = new DoublePreference("Vision/Jump Limit", TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*0.02);
  protected static DoublePreference omegaPenalty = new DoublePreference("Vision/Omega Penalty", VisionConstants.OMEGA_PENALTY);
}
