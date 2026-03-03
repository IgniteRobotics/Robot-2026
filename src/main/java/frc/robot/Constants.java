// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.statemachines.AllianceState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class RobotConstants {
    public static final Distance robotCenterToEdge = Meters.of(0.47);
    public static final Distance robotCenterToIntakeExtended = Meters.of(0.65);
  }

  public static class FieldConstants {

    public static final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /* ***** SHOOTING TARGETS ***** */

    public static final Pose3d BLUE_HUB =
        new Pose3d(
            Meters.of(4.623),
            Meters.of(4.041),
            Meters.of(1.435),
            Rotation3d.kZero);
    public static final Pose3d BLUE_LEFT_PASS =
        new Pose3d(
            Meters.of(7),
            Meters.of(1),
            Meters.of(0),
            Rotation3d.kZero);
    public static final Pose3d BLUE_RIGHT_PASS =
        new Pose3d(
            Meters.of(1.0),
            Meters.of(1.0),
            Meters.of(0.0),
            Rotation3d.kZero);

    public static final Pose3d RED_HUB =
        new Pose3d(
            Meters.of(12.276),
            Meters.of(4.041),
            Meters.of(1.435),
            Rotation3d.kZero);

    public static final Pose3d RED_LEFT_PASS =
        new Pose3d(
            Meters.of(14),
            Meters.of(1),
            Meters.of(0),
            Rotation3d.kZero);
    public static final Pose3d RED_RIGHT_PASS =
        new Pose3d(
            Meters.of(14),
            Meters.of(7),
            Meters.of(0),
            Rotation3d.kZero);

    /* ***** DRIVING POSES ***** */
    public static final Pose3d BLUE_OUTPOST_LOADING =
        new Pose3d(
            RobotConstants.robotCenterToIntakeExtended,
            RobotConstants.robotCenterToEdge,
            Meters.of(0.0),
            new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(180.0)));

    public static final Pose3d RED_OUTPOST_LOADING =
        new Pose3d(
            layout.getOrigin().getMeasureX().minus(RobotConstants.robotCenterToIntakeExtended),
            layout.getOrigin().getMeasureY().minus(RobotConstants.robotCenterToEdge),
            Meters.of(0),
            Rotation3d.kZero);

    /* ***** Helpers ***** */
    public static final Pose3d getHubTarget() {
      return AllianceState.getInstance().getAlliance().equals(Alliance.Blue) ? BLUE_HUB : RED_HUB;
    }

    public static final Pose3d getLeftPassTarget() {
      return AllianceState.getInstance().getAlliance().equals(Alliance.Blue)
          ? BLUE_LEFT_PASS
          : RED_LEFT_PASS;
    }

    public static final Pose3d getRightPassTarget() {
      return AllianceState.getInstance().getAlliance().equals(Alliance.Blue)
          ? BLUE_RIGHT_PASS
          : RED_RIGHT_PASS;
    }

    public static final Pose3d getOutpostLoading() {
      return AllianceState.getInstance().getAlliance().equals(Alliance.Blue)
          ? BLUE_OUTPOST_LOADING
          : RED_OUTPOST_LOADING;
    }
  }
}
