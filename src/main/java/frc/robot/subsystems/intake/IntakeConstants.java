package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {

  private IntakeConstants() {}

  public static final int ROLLER_MOTOR_ID = 2;
  public static final int EXTENSION_MOTOR_ID = 1;
  public static final int ROLLER_FOLLOWER_MOTOR_ID = 10;

  // TODO: Tune Roller and Extension Motors

  // Roller Motor
  public static final double ROLLER_KS = 0;
  public static final double ROLLER_KV = 0;
  public static final double ROLLER_KP = 0;
  public static final double ROLLER_KD = 0;

  public static MotorOutputConfigs createRotorLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static MotorOutputConfigs createRotorFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static Slot0Configs createRollerMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = ROLLER_KS;
    slot.kV = ROLLER_KV;
    slot.kP = ROLLER_KP;
    slot.kD = ROLLER_KD;
    return slot;
  }

  public static CurrentLimitsConfigs createRollerMotorCurrentLimitsConfigs() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimitEnable = true;
    config.StatorCurrentLimit = 40.0;
    return config;
  }

  // Extension Motor
  public static final double ALLOWABLE_EXTENSION_ERROR = 0.1;
  public static final double INTAKE_FORWARD_LIMIT = 14.55;
  public static final double INTAKE_REVERSE_LIMIT = 0.1;
  public static final double EXTENSION_KS = 0;
  public static final double EXTENSION_KP = 6.0;
  public static final double EXTENSION_KD = 0.2;

  public static Slot0Configs createExtensionMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = EXTENSION_KS;
    slot.kP = EXTENSION_KP;
    slot.kD = EXTENSION_KD;
    return slot;
  }

  public static SoftwareLimitSwitchConfigs createExtensionSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = true;
    configs.ReverseSoftLimitEnable = true;
    configs.ForwardSoftLimitThreshold = INTAKE_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = INTAKE_REVERSE_LIMIT;
    return configs;
  }

  public static MotorOutputConfigs createExtensionMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    // newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Coast;
    return newConfigs;
  }

  public static MotionMagicConfigs creatrExtenstionMotionMagicConfigs() {
    MotionMagicConfigs newConfigs = new MotionMagicConfigs();
    newConfigs.MotionMagicCruiseVelocity = 40;
    newConfigs.MotionMagicAcceleration = 60;
    newConfigs.MotionMagicJerk = 400;
    return newConfigs;
  }

  public static final double SAFE_HOMING_EFFORT = -0.2;
  public static final double SAFE_STATOR_LIMIT = 0.8;
}
