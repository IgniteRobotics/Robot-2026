package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

  private ShooterConstants() {}

  public static final int FLYWHEEL_LEFT_LEADER_MOTOR_ID = 5;
  public static final int FLYWHEEL_RIGHT_MOTOR_ID = 6;
  public static final int FLYWHEEL_LEFT_FOLLOWER_MOTOR_ID = 7;
  public static final int HOOD_MOTOR_ID = 8;

  public static MotorOutputConfigs createRightFlywheelMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static MotorOutputConfigs createLeftFlywheelLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static MotorOutputConfigs createLeftFlywheelFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static MotorOutputConfigs createHoodMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final double HOOD_FORWARD_LIMIT = 4.9;
  public static final double HOOD_REVERSE_LIMIT = 0;

  public static SoftwareLimitSwitchConfigs createHoodSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = true;
    configs.ReverseSoftLimitEnable = true;
    configs.ForwardSoftLimitThreshold = HOOD_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = HOOD_REVERSE_LIMIT;
    return configs;
  }

  public static final double FLYWHEEL_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createFlywheelCurrentLimitsConfigs() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.StatorCurrentLimit = FLYWHEEL_CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = true;
    return configs;
  }

  public static final double HOOD_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createHoodCurrentLimitsConfigs() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.StatorCurrentLimit = HOOD_CURRENT_LIMIT;
    configs.StatorCurrentLimitEnable = true;
    return configs;
  }

  public static final double HOOD_KS = 4.8;
  public static final double HOOD_KP = 9;
  public static final double HOOD_KD = 0;

  public static Slot0Configs createHoodMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = HOOD_KS;
    slot.kP = HOOD_KP;
    slot.kD = HOOD_KD;
    return slot;
  }


  // TODO: Tune Flywheel and Hood Motor

  // Flywheel motor
  public static final double FLYWHEEL_KV = 0.12807;
  public static final double FLYWHEEL_KA = 0.020039;
  public static final double FLYWHEEL_KP = 0.17969;

  public static Slot0Configs createFlywheelMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kV = FLYWHEEL_KV;
    slot.kP = FLYWHEEL_KP;
    slot.kA = FLYWHEEL_KA;
    return slot;
  }


  /* 
  public static final double ALLOWABLE_HOOD_ERROR = 0.1;
  public static final double HOOD_FORWARD_LIMIT = 6.2;
  public static final double HOOD_REVERSE_LIMIT = 0;

  public static SoftwareLimitSwitchConfigs createHoodSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = true;
    configs.ReverseSoftLimitEnable = true;
    configs.ForwardSoftLimitThreshold = HOOD_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = HOOD_REVERSE_LIMIT;
    return configs;
  }

  public static MotorOutputConfigs createHoodMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }
  

  public static final DutyCycleOut SAFE_HOMING_EFFORT = new DutyCycleOut(-0.2);
  public static final Current SAFE_STATOR_LIMIT = Amp.of(0.8);

  */
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
}
