// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    Epilogue.configure(
        config -> {
          // Default backend is NetworkTables - no need to set config.backend
          config.root = "Robot";
        });

    // Start DataLogManager to capture NetworkTables data to disk (.wpilog files)
    // This provides post-match analysis capability for ALL telemetry
    DataLogManager.start();

    // Start Phoenix 6 SignalLogger for high-fidelity CTRE device logging (.hoot
    // files)
    // This captures ALL Phoenix 6 status signals at full CAN rate with timestamps
    SignalLogger.start();

    // Bind Epilogue to robot loop - runs at 50Hz, phase-offset from main loop
    Epilogue.bind(this);

    StringLogEntry metaData = new StringLogEntry(DataLogManager.getLog(), "MetaData");
    metaData.append("Project Name: " + BuildConstants.MAVEN_NAME);
    metaData.append("Build Date: " + BuildConstants.BUILD_DATE);
    metaData.append("Commit Hash: " + BuildConstants.GIT_SHA);
    metaData.append("Git Date: " + BuildConstants.GIT_DATE);
    metaData.append("Git Branch: " + BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        metaData.append("GitDirty: " + "All changes commited");
        break;
      case 1:
        metaData.append("GitDirty: " + "Uncomitted changes");
        break;
      default:
        metaData.append("GitDirty: " + "Unknown");
        break;
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.configureSubsystemDefaultCommands();
    m_robotContainer.configureTeleopBindings();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    SignalLogger.stop();
    m_robotContainer.removeSubsystemDefaultCommands();
    m_robotContainer.configureTestBindings();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
