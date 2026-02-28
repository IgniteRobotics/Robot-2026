package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

  private ShooterConstants() {}

  // TODO: Change to actual ports
  public static final int FLYWHEEL_MOTOR_ID = 4;
  public static final int HOOD_MOTOR_ID = 5;

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
  public static final double HOOD_FORWARD_LIMIT = 100;
  public static final double HOOD_REVERSE_LIMIT = 0;
  public static final double HOOD_KS = 0;
  public static final double HOOD_KP = 0;
  public static final double HOOD_KD = 0;

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

  public static final DutyCycleOut SAFE_HOMING_EFFORT = new DutyCycleOut(-0.2);
  public static final Current SAFE_STATOR_LIMIT = Amp.of(0.8);

  // Conversion Constants
  public static final Angle ROTATIONS_PER_LAUNCH_DEGREE =
      Rotations.of(1); // TODO: Get Better Estimate
  public static final Distance FLYWHEEL_RADIUS = Inch.of(1); // TODO: Get Better Estimate

  // Lemon Yeeting Constants
  public static final Distance SHOOTER_HEIGHT = Inch.of(65); // TODO: Get Better Estimate
  public static final Distance HUB_HEIGHT = Inch.of(23); // TODO: Get Better Estimate

  public static final Distance FROM_HUB_CENTER_TO_WALL = Inch.of(23.5);
  public static final Angle MIN_HOOD_ANGLE = Degrees.of(0); // TODO: Get Better Estimate
  public static final double OPTIMAL_PASSING_ENTRY_SLOPE = -1; // TODO: Tune
  public static final double OPTIMAL_HUB_ENTRY_SLOPE = -1; // TODO: Tune
}
