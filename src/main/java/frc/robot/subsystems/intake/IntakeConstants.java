package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

public class IntakeConstants {

  private IntakeConstants() {}

  public static final int ROLLER_MOTOR_ID = 2;
  public static final int EXTENSION_MOTOR_ID = 1;
  public static final int ROLLER_FOLLOWER_MOTOR_ID = 10;
  public static final int EXTENSION_FOLLOWER_MOTOR_ID = 14;

  public static MotorOutputConfigs createRollerLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static MotorOutputConfigs createRollerFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final double ROLLER_STATOR_CURRENT_LIMIT = 80;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createRollerMotorCurrentLimitsConfigs() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimitEnable = true;
    config.SupplyCurrentLimitEnable = true;
    config.StatorCurrentLimit = ROLLER_STATOR_CURRENT_LIMIT;
    config.SupplyCurrentLimit = ROLLER_SUPPLY_CURRENT_LIMIT;
    return config;
  }

  public static OpenLoopRampsConfigs createRollerMotorRampConfigs() {
    OpenLoopRampsConfigs configs = new OpenLoopRampsConfigs();
    configs.VoltageOpenLoopRampPeriod = 1.5; // 1.5 secs to get from 0V to 12V
    configs.DutyCycleOpenLoopRampPeriod = 1.5;
    return configs;
  }

  // Extension Motor
  public static final double ALLOWABLE_EXTENSION_ERROR = 0.5;

  public static final double EXTENSION_KS = 0.325;
  public static final double EXTENSION_KP = 4;
  public static final double EXTENSION_KI = 0.0;
  public static final double EXTENSION_KD = 0.33;

  public static Slot0Configs createExtensionMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = EXTENSION_KS;
    slot.kP = EXTENSION_KP;
    slot.kD = EXTENSION_KD;
    return slot;
  }

  // Fix PID values for springiness
  public static final double EXTENSION_SPRINGY_KP = 0;
  public static final double EXTENSION_SPRINGY_KD = 0;

  public static Slot1Configs createExtensionMotorSlot1Configs() {
    Slot1Configs slot = new Slot1Configs();
    slot.kS = EXTENSION_KS;
    slot.kP = IntakePreferences.springykP.getValue();
    slot.kD = IntakePreferences.springykD.getValue();
    return slot;
  }

  public static final double INTAKE_FORWARD_LIMIT = 14.6;
  public static final double INTAKE_REVERSE_LIMIT = 3.5;

  public static SoftwareLimitSwitchConfigs createExtensionSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = true;
    configs.ReverseSoftLimitEnable = true;
    configs.ForwardSoftLimitThreshold = INTAKE_FORWARD_LIMIT;
    configs.ReverseSoftLimitThreshold = INTAKE_REVERSE_LIMIT;
    return configs;
  }

  public static MotorOutputConfigs createExtensionLeaderMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static MotorOutputConfigs createExtensionFollowerMotorOutputConfigs() {
    MotorOutputConfigs newConfigs = new MotorOutputConfigs();
    newConfigs.Inverted = InvertedValue.Clockwise_Positive;
    newConfigs.NeutralMode = NeutralModeValue.Brake;
    return newConfigs;
  }

  public static final double EXTENSION_STATOR_CURRENT_LIMIT = 75;
  public static final double EXTENSION_SUPPLY_CURRENT_LIMIT = 40;

  public static CurrentLimitsConfigs createExtenstionMotorCurrentLimitsConfigs() {
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    config.StatorCurrentLimitEnable = true;
    config.StatorCurrentLimit = EXTENSION_STATOR_CURRENT_LIMIT;
    config.SupplyCurrentLimitEnable = true;
    config.SupplyCurrentLimit = EXTENSION_SUPPLY_CURRENT_LIMIT;
    return config;
  }

  public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.5;

  public static ClosedLoopRampsConfigs creatClosedLoopRampsConfigs() {
    ClosedLoopRampsConfigs config = new ClosedLoopRampsConfigs();
    config.VoltageClosedLoopRampPeriod = VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;
    return config;
  }

  public static final Measure<CurrentUnit> COMPLIANT_RESISTANCE_CURRENT_LIMIT = Units.Amp.of(8);

  public static final double SAFE_HOMING_EFFORT = -0.2;
  public static final double SAFE_STATOR_LIMIT = 100;

  public static final double INTAKING_SETPOINT = 14.25;
  public static final double START_ROLLER_SETPOINT = 9.0;
}
