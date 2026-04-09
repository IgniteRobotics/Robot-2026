package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.generated.TunerConstants;
import frc.robot.preferences.DoublePreference;

final class VisionPreferences {

  private VisionPreferences() {}

  protected static DoublePreference xyStdDevCoef =
      new DoublePreference("Vision/xyStdDevCoef", VisionConstants.XY_STD_DEV_COEFFICIENT);
  protected static DoublePreference thetaStdDevCoef =
      new DoublePreference("Vision/thetaStdDevCoef", VisionConstants.THETA_STD_DEV_COEFFICIENT);
  protected static DoublePreference jumpLimit =
      new DoublePreference(
          "Vision/Jump Limit", TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.02);
  protected static DoublePreference omegaPenalty =
      new DoublePreference("Vision/Omega Penalty", VisionConstants.OMEGA_PENALTY);

  // Clustering and Drift Detection Preferences
  protected static DoublePreference clusterDistanceThreshold =
      new DoublePreference(
          "Vision/Clustering/Distance Threshold", VisionConstants.CLUSTER_DISTANCE_THRESHOLD);
  protected static DoublePreference clusterAngleThreshold =
      new DoublePreference(
          "Vision/Clustering/Angle Threshold", VisionConstants.CLUSTER_ANGLE_THRESHOLD);
  protected static DoublePreference trustScalingFactor =
      new DoublePreference(
          "Vision/Clustering/Trust Scaling Factor", VisionConstants.TRUST_SCALING_FACTOR);
  protected static DoublePreference driftDetectionCycles =
      new DoublePreference("Vision/Drift/Detection Cycles", VisionConstants.DRIFT_DETECTION_CYCLES);
  protected static DoublePreference driftDirectionTolerance =
      new DoublePreference(
          "Vision/Drift/Direction Tolerance", VisionConstants.DRIFT_DIRECTION_TOLERANCE);
  protected static DoublePreference driftMinimumDistance =
      new DoublePreference("Vision/Drift/Minimum Distance", VisionConstants.DRIFT_MINIMUM_DISTANCE);
  protected static DoublePreference driftConvergenceThreshold =
      new DoublePreference(
          "Vision/Drift/Convergence Threshold", VisionConstants.DRIFT_CONVERGENCE_THRESHOLD);
}
