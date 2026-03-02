package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX flywheelMotor;
  private final TalonFX hoodMotor;

  @Logged(name = "Velocity Target", importance = Importance.CRITICAL)
  private AngularVelocity velocityTarget; // Rotations Per Second

  private VelocityVoltage velocityControl;

  @Logged(name = "Hood Target", importance = Importance.CRITICAL)
  private Angle hoodTarget; // Rotations

  private PositionTorqueCurrentFOC hoodControl;

  public class LaunchRequest {
    private Angle launchHoodTarget;
    private AngularVelocity launchVelocity;

    public LaunchRequest(Angle theta, LinearVelocity velocity) {
      // hood angle in degrees (0 degrees is all the way back) is converted to motor rotations
      launchHoodTarget =
          Rotations.of(
              theta.in(Degrees) * ShooterConstants.ROTATIONS_PER_LAUNCH_DEGREE.in(Rotations));
      // meters per second is converted to rotations per second of the flywheel
      launchVelocity =
          RotationsPerSecond.of(
              velocity.in(MetersPerSecond)
                  / (2 * Math.PI * ShooterConstants.FLYWHEEL_RADIUS.in(Meters)));
    }

    public Angle getHoodTarget() {
      return launchHoodTarget;
    }

    public AngularVelocity getVelocityTarget() {
      return launchVelocity;
    }
  }

  public ShooterSubsystem() {
    flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID);
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

    flywheelMotor.getConfigurator().apply(ShooterConstants.createFlywheelMotorSlot0Configs());
    velocityTarget = RotationsPerSecond.of(0);
    velocityControl = new VelocityVoltage(0);

    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodMotorSlot0Configs());
    hoodMotor.getConfigurator().apply(ShooterConstants.createHoodSoftwareLimitSwitchConfigs());
    hoodTarget = Rotations.of(0);
    hoodControl = new PositionTorqueCurrentFOC(0);
  }

  @Override
  public void periodic() {
    flywheelMotor.setControl(velocityControl.withVelocity(velocityTarget.in(RotationsPerSecond)));
    hoodMotor.setControl(hoodControl.withVelocity(hoodTarget.in(Rotations)));
  }

  public Command spinFlywheelCommand() {
    return runOnce(
            () ->
                velocityTarget =
                    RotationsPerSecond.of(ShooterPreferences.flywheelLaunchSpeed.getValue()))
        .withName("Start Spinning Flywheel");
  }

  public Command stopFlywheelCommand() {
    return runOnce(() -> velocityTarget = RotationsPerSecond.of(0))
        .withName("Stop Spinning Flywheel");
  }

  @Logged(name = "At Hood Setpoint", importance = Importance.CRITICAL)
  public boolean atHoodSetpoint() {
    return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodTarget.in(Rotations))
        < IntakeConstants.ALLOWABLE_EXTENSION_ERROR;
  }

  public Command setHoodCommand(Angle position) {
    return runOnce(() -> hoodTarget = position)
        .andThen(Commands.waitUntil(() -> atHoodSetpoint()))
        .withName("Set Hood Angle");
  }

  public Command launchLemonsCommand() {
    return setHoodCommand(Rotations.of(ShooterPreferences.hoodLaunchAngle.getValue()))
        .andThen(spinFlywheelCommand())
        .withName("Start Launching Lemons");
  }

  public Command stowCommand() {
    return stopFlywheelCommand().andThen(setHoodCommand(Rotations.of(0))).withName("Stow Shooter");
  }

  public Command homeShooterCommand() {
    return runEnd(
            () -> hoodMotor.set(ShooterConstants.SAFE_HOMING_EFFORT.Output),
            () -> hoodMotor.setPosition(0))
        .until(
            () -> {
              return hoodMotor.getStatorCurrent().getValueAsDouble()
                  > ShooterConstants.SAFE_STATOR_LIMIT.in(Amp);
            });
  }

  // helper method to find, given a distance from the robot to the tag,
  // (1) necessary angle of the hood
  // (2) necessary velocity of flywheel
  // to land a lemon in the goal
  public LaunchRequest createLaunchRequest(double distanceToTag) {
    double y1 = ShooterConstants.SHOOTER_HEIGHT.in(Meters);
    double x2 = distanceToTag + ShooterConstants.OFFSET_DISTANCE.in(Meters);
    double y2 = ShooterConstants.GOAL_HEIGHT.in(Meters);

    double slope = ShooterConstants.OPTIMAL_ENTRY_SLOPE;
    double a, b, vertex;
    Angle theta, motorAngle;
    do {
      // system of equations
      // (y2) = a(x2*x2) + b(x2) + y1
      // slope = 2a(x2) + b
      a = (slope * x2 + y1 - y2) / (x2 * x2);
      b = (slope - 2 * a * x2);
      theta = Radians.of(Math.atan(b)); // launch angle (Hood Angle Conversion: MATH.PI/2 - theta)
      motorAngle = Radians.of(Math.PI / 2 - theta.in(Radians));
      vertex = -1 * b / (2 * a);
      slope -= 0.05;
    } while ((vertex > x2 - ShooterConstants.MIN_VERTEX_DISTANCE.in(Meters))
        && !(motorAngle.in(Degrees) < ShooterConstants.MIN_HOOD_ANGLE.in(Degrees)));

    if (motorAngle.in(Degrees) < ShooterConstants.MIN_HOOD_ANGLE.in(Degrees)) return null;

    // system of equations
    // (-b/2a) = (velocity)*cos(theta)*t
    // 2g(t) = (velocity)*sin(theta)
    LinearVelocity velocity =
        MetersPerSecond.of(
            Math.sqrt(
                2 * 9.8 * vertex / (Math.sin(theta.in(Radians)) * Math.cos(theta.in(Radians)))));

    return new LaunchRequest(theta, velocity);
  }
}
