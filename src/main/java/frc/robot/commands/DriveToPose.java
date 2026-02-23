package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.statemachines.DriveState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DrivePreferences;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private DrivetrainSubsystem drivetrain;
  private DriveState driveState;
  private Supplier<Pose2d> targetPose;
  private PIDController xyControl;
  private ProfiledPIDController rotControl;
  private TrapezoidProfile driveProfile;
  private Time lastTime;

  private double driveErrorMag = 0.0;
  private double rotErrorMag = 0.0;

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;

  public DriveToPose(DrivetrainSubsystem subsystem, Supplier<Pose2d> pose) {
    drivetrain = subsystem;
    targetPose = pose;
    driveState = DriveState.getInstance();
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xyControl = drivetrain.getTranslationPIDController();
    rotControl = drivetrain.getRotationPIDController();

    Pose2d currentPose = driveState.getCurrentDriveStats().Pose;
    Pose2d targetPose = this.targetPose.get();
    ChassisSpeeds fieldVelocity = driveState.getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DrivePreferences.autopilotMaxVelocity.get(),
                DrivePreferences.autopilotMaxAccel.get()));
    rotControl.setConstraints(
        new TrapezoidProfile.Constraints(
            DrivePreferences.autopilotMaxRotation.get(),
            DrivePreferences.autopilotMaxAngularAccel.get()));

    xyControl.reset();
    rotControl.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Time.ofBaseUnits(Timer.getTimestamp(), Seconds); // Timer.getTimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = driveState.getCurrentDriveStats().Pose;
    Pose2d targetPose = this.targetPose.get();

    // find the error between were the robot is, and where the robot isn't.
    Pose2d poseError = currentPose.relativeTo(targetPose);
    // magnitudes
    driveErrorMag = poseError.getTranslation().getNorm();
    rotErrorMag = Math.abs(poseError.getRotation().getRadians());

    // find your feed forward scaling factors
    double linearFFScaler =
        MathUtil.clamp(
            (driveErrorMag - DrivePreferences.autopilotLinearFFMinError.get())
                / (DrivePreferences.autopilotLinearFFMaxError.get()
                    - DrivePreferences.autopilotLinearFFMinError.get()),
            0.0,
            1.0);
    double rotFFScaler =
        MathUtil.clamp(
            (Units.radiansToRotations(rotErrorMag)
                    - DrivePreferences.autopilotRotationFFMinError.get())
                / (DrivePreferences.autopilotRotationFFMaxError.get()
                    - DrivePreferences.autopilotRotationFFMinError.get()),
            0.0,
            1.0);

    // get a vector of where we're going
    var driveVector = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    // calculate speed
    double setpointSpeed =
        driveVector.norm()
                <= DriveConstants
                    .TRANSLATION_ALIGN_TOLERANCE // Don't calculate velocity in direction when
            // really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(driveVector) / driveVector.norm();
    setpointSpeed = Math.max(setpointSpeed, DrivePreferences.autopilotMinAppliedVelocity.get());

    // find the next setpoint
    State driveSetpoint =
        driveProfile.calculate(
            Constants.RobotContants.LOOP_PERIOD_SECONDS,
            new State(
                driveVector.norm(), -setpointSpeed), // Use negative as profile has zero at target
            new State(0.0, 0.0));

    // calculate final speed
    double scaledSpeed =
        xyControl.calculate(driveErrorMag, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (scaledSpeed < xyControl.getErrorTolerance()) scaledSpeed = 0.0;

    // calculate turn
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

    // output drive vector
    Translation2d driveVelocity = new Translation2d(scaledSpeed, targetToCurrentAngle);
    // track for next iteration
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // calculate rotation speed
    double rotSetpointSpeed =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime.in(Seconds))
            : rotControl.getSetpoint().velocity;

    // final scaled speed
    double scaledRotSpeed =
        rotControl.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), rotSetpointSpeed))
            + rotControl.getSetpoint().velocity * rotFFScaler;
    if (rotErrorMag < rotControl.getPositionTolerance()) scaledRotSpeed = 0.0;

    // store for next iteration and update time
    lastGoalRotation = targetPose.getRotation();
    lastTime = Time.ofBaseUnits(Timer.getTimestamp(), Seconds);

    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST
            .withVelocityX(driveVelocity.getX())
            .withVelocityY(driveVelocity.getY())
            .withRotationalRate(scaledRotSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        DriveConstants.AUTO_DRIVE_REQUEST.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveErrorMag <= xyControl.getErrorTolerance()
        && rotErrorMag <= rotControl.getPositionTolerance();
  }
}
