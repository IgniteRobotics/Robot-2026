package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

  private ShooterConstants() {}

  public static final int FLYWHEEL_LEADER_MOTOR_ID = 5;
  public static final int FLYWHEEL_FOLLOWER_MOTOR_ID = 6;
  public static final int HOOD_MOTOR_ID = 4;

  public static MotorOutputConfigs createLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static MotorOutputConfigs createFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  // TODO: Tune Flywheel and Hood Motor

  // Flywheel motor
  public static final double FLYWHEEL_KS = 0.11239;
  public static final double FLYWHEEL_KV = 0.11589;
  public static final double FLYWHEEL_KA = 0.0034356;
  public static final double FLYWHEEL_KP = 0.066945 * 2;

  public static Slot0Configs createFlywheelMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = FLYWHEEL_KS;
    slot.kV = FLYWHEEL_KV;
    slot.kP = FLYWHEEL_KP;
    slot.kA = FLYWHEEL_KA;
    return slot;
  }

  public static final double ALLOWABLE_HOOD_ERROR = 0.1;
  public static final double HOOD_FORWARD_LIMIT = 5.8;
  public static final double HOOD_REVERSE_LIMIT = 0;
  public static final double HOOD_KS = 0;
  public static final double HOOD_KP = ShooterPreferences.hoodkP.getValue();
  public static final double HOOD_KD = ShooterPreferences.hoodkD.getValue();

  public static Slot0Configs createHoodMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = HOOD_KS;
    slot.kP = HOOD_KP;
    slot.kD = HOOD_KD;
    return slot;
  }

  public static SoftwareLimitSwitchConfigs createHoodSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = false;
    configs.ReverseSoftLimitEnable = false;
    configs.ForwardSoftLimitThreshold = HOOD_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = HOOD_REVERSE_LIMIT;
    return configs;
  }

  public static MotorOutputConfigs createHoodMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final DutyCycleOut SAFE_HOMING_EFFORT = new DutyCycleOut(-0.2);
  public static final Current SAFE_STATOR_LIMIT = Amp.of(0.8);

  // Conversion Constants
  public static final Angle ROTATIONS_PER_LAUNCH_DEGREE = Rotations.of(0.2);
  public static final Distance FLYWHEEL_RADIUS = Inch.of(2);

  // Lemon Yeeting Constants
  public static final Distance SHOOTER_HEIGHT = Inch.of(25.5);
  public static final Distance HUB_HEIGHT = Inch.of(71.5);
  public static final Distance FROM_HUB_CENTER_TO_WALL = Inch.of(23.5);
  public static final Angle MIN_HOOD_ANGLE = Degrees.of(20);
  public static final double OPTIMAL_PASSING_ENTRY_SLOPE = -1; // TODO: Tune
  public static final double OPTIMAL_HUB_ENTRY_SLOPE = -1; // TODO: Tune

  // TODO:  verify these!
  // funnel poses.
  public static final Pose3d BLUE_TARGET =
      new Pose3d(
          Distance.ofBaseUnits(4.623, Meters),
          Distance.ofBaseUnits(4.041, Meters),
          Distance.ofBaseUnits(1.435, Meters),
          Rotation3d.kZero);
  public static final Pose3d RED_TARGET =
      new Pose3d(
          Distance.ofBaseUnits(11.976, Meters), // 12.276
          Distance.ofBaseUnits(4.041, Meters),
          Distance.ofBaseUnits(1.435, Meters),
          Rotation3d.kZero);
}
