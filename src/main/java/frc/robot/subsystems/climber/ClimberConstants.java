package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

final class ClimberConstants {

  private ClimberConstants() {}

  // TODO move to real ID when implemented.
  protected static final int CLIMB_MOTOR_ID = 19;

  // TODO: Tune motor
  protected static final Angle ALLOWABLE_CLIMB_ERROR = Rotations.of(0.1);
  protected static final Angle CLIMB_FORWARD_LIMIT = Rotations.of(100);
  protected static final Angle CLIMB_REVERSE_LIMIT = Rotations.of(0);
  protected static final double CLIMB_KS = 0;
  protected static final double CLIMB_KP = 0;
  protected static final double CLIMB_KD = 0;

  protected static Slot0Configs createClimbMotorSlot0Configs() {
    Slot0Configs slot = new Slot0Configs();
    slot.kS = CLIMB_KS;
    slot.kP = CLIMB_KP;
    slot.kD = CLIMB_KD;
    return slot;
  }

  protected static SoftwareLimitSwitchConfigs createClimbSoftwareLimitSwitchConfigs() {
    SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
    configs.ForwardSoftLimitEnable = false;
    configs.ReverseSoftLimitEnable = false;
    configs.ForwardSoftLimitThreshold = CLIMB_FORWARD_LIMIT.in(Rotations);
    configs.ReverseSoftLimitThreshold = CLIMB_REVERSE_LIMIT.in(Rotations);
    return configs;
  }

  protected static final DutyCycleOut SAFE_HOMING_EFFORT = new DutyCycleOut(-0.2);
  protected static final Current SAFE_STATOR_LIMIT = Amp.of(0.8);
}
