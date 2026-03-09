package frc.robot.subsystems.ui;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.ShiftState;

@Logged
public class UISubsystem extends SubsystemBase {

  // Controllers for rumble feedback
  private final GenericHID driverController;
  private final GenericHID operatorController;

  // State tracking
  enum FeedbackState {
    GREEN_SOLID, // Hub active, OK to shoot
    RED_SOLID, // Hub inactive, NOT OK to shoot
    YELLOW_PULSING, // Warning period before transition
    FAST_FLASH // Brief flash when transition occurs
  }

  private FeedbackState currentState = FeedbackState.GREEN_SOLID;
  private boolean lastCanScore = true;
  private boolean manualOverride = false; // When true, periodic rumble is disabled

  // Cycle counters for animations
  private int cycleCounter = 0;
  private int flashCycleCounter = 0;

  /**
   * Creates a new UISubsystem.
   *
   * @param driverController The driver's controller for rumble feedback
   * @param operatorController The operator's controller for rumble feedback
   */
  public UISubsystem(GenericHID driverController, GenericHID operatorController) {
    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  @Override
  public void periodic() {
    boolean canScore = ShiftState.getInstance().isOurHubActive();
    double timeToNextTransition = getTimeUntilNextTransition();

    // Detect state transitions
    if (canScore != lastCanScore) {
      // Transition just happened
      currentState = FeedbackState.FAST_FLASH;
      flashCycleCounter = 0;
    } else if (currentState == FeedbackState.FAST_FLASH) {
      // Check if fast flash duration is complete
      flashCycleCounter++;
      if (flashCycleCounter >= UIConstants.FAST_FLASH_DURATION_CYCLES) {
        currentState = canScore ? FeedbackState.GREEN_SOLID : FeedbackState.RED_SOLID;
      }
    } else {
      // Check if we should enter warning period
      double warningPeriod =
          canScore
              ? UIPreferences.warningActiveToInactive.getValue()
              : UIPreferences.warningInactiveToActive.getValue();

      if (timeToNextTransition > 0 && timeToNextTransition <= warningPeriod) {
        if (currentState != FeedbackState.YELLOW_PULSING) {
          currentState = FeedbackState.YELLOW_PULSING;
        }
      } else {
        // Normal solid color
        currentState = canScore ? FeedbackState.GREEN_SOLID : FeedbackState.RED_SOLID;
      }
    }

    // Publish color to dashboard
    String color = getColorForState();
    SmartDashboard.putString("UI/ScoringStatusColor", color);

    // Update rumble based on current state
    updateRumble();

    // Update cycle counter
    cycleCounter++;

    // Update last state
    lastCanScore = canScore;
  }

  /**
   * Gets the time in seconds until the next scoring window transition. Returns -1 if in a period
   * where both hubs are active (no transition expected).
   */
  private double getTimeUntilNextTransition() {
    ShiftState.Shift currentShift = ShiftState.getInstance().getShiftFromMatchTime();
    double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();

    switch (currentShift) {
      case AUTONOMOUS:
        // Transition to TRANSITION period at TELEOP_START_TIME
        return matchTime - ShiftState.TELEOP_START_TIME;

      case TRANSITION:
        // Transition to SHIFT_1 at TRANSITION_END_TIME
        return matchTime - ShiftState.TRANSITION_END_TIME;

      case SHIFT_1:
        return matchTime - ShiftState.SHIFT_1_END_TIME;

      case SHIFT_2:
        return matchTime - ShiftState.SHIFT_2_END_TIME;

      case SHIFT_3:
        return matchTime - ShiftState.SHIFT_3_END_TIME;

      case SHIFT_4:
        return matchTime - ShiftState.SHIFT_4_END_TIME;

      case ENDGAME:
        // No more transitions in endgame
        return -1;

      default:
        return -1;
    }
  }

  /** Returns the color hex code based on current feedback state. */
  private String getColorForState() {
    switch (currentState) {
      case GREEN_SOLID:
        return UIConstants.COLOR_GREEN;

      case RED_SOLID:
        return UIConstants.COLOR_RED;

      case YELLOW_PULSING:
        // Toggle between yellow and black at ~2Hz
        boolean isOn = (cycleCounter / UIConstants.YELLOW_PULSE_TOGGLE_CYCLES) % 2 == 0;
        return isOn ? UIConstants.COLOR_YELLOW : UIConstants.COLOR_BLACK;

      case FAST_FLASH:
        // Toggle between target color and black at 5Hz
        boolean flashOn = (flashCycleCounter / UIConstants.FAST_FLASH_TOGGLE_CYCLES) % 2 == 0;
        String targetColor = lastCanScore ? UIConstants.COLOR_GREEN : UIConstants.COLOR_RED;
        return flashOn ? targetColor : UIConstants.COLOR_BLACK;

      default:
        return UIConstants.COLOR_BLACK;
    }
  }

  /**
   * Gets the rumble intensity for endgame warnings. Returns -1 if no endgame rumble should occur.
   */
  private double getEndgameRumbleIntensity() {
    double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();

    // If not in a match or time is invalid, no endgame rumble
    if (matchTime < 0) {
      return -1;
    }

    double slowPulseThreshold = UIPreferences.endgameSlowPulse.getValue();
    double fastPulseThreshold = UIPreferences.endgameFastPulse.getValue();

    if (matchTime <= fastPulseThreshold) {
      // Fast pulse (5Hz) at 75% intensity
      boolean on = (cycleCounter / UIConstants.RUMBLE_ENDGAME_FAST_TOGGLE_CYCLES) % 2 == 0;
      return on ? UIConstants.RUMBLE_ENDGAME_INTENSITY : 0.0;
    } else if (matchTime <= slowPulseThreshold) {
      // Slow pulse (2Hz) at 75% intensity
      boolean on = (cycleCounter / UIConstants.RUMBLE_ENDGAME_SLOW_TOGGLE_CYCLES) % 2 == 0;
      return on ? UIConstants.RUMBLE_ENDGAME_INTENSITY : 0.0;
    }

    return -1; // No endgame rumble
  }

  /**
   * Updates rumble pattern based on current feedback state. Pulses rumble at same frequency as
   * visual feedback. Endgame rumble takes priority over shift-based rumble. Skipped if manual
   * override is active.
   */
  private void updateRumble() {
    // Skip automatic rumble if manual override is active
    if (manualOverride) {
      return;
    }

    double intensity = 0.0;

    // Check endgame rumble first (priority)
    double endgameIntensity = getEndgameRumbleIntensity();
    if (endgameIntensity >= 0) {
      intensity = endgameIntensity;
    } else {
      // Normal shift-based rumble
      switch (currentState) {
        case YELLOW_PULSING:
          // Pulse at 2Hz with 50% intensity (matches yellow pulsing)
          boolean warningOn = (cycleCounter / UIConstants.RUMBLE_WARNING_TOGGLE_CYCLES) % 2 == 0;
          intensity = warningOn ? UIConstants.RUMBLE_WARNING_INTENSITY : 0.0;
          break;

        case FAST_FLASH:
          // Pulse at 5Hz with 50% intensity (matches fast flash)
          boolean flashOn =
              (flashCycleCounter / UIConstants.RUMBLE_TRANSITION_TOGGLE_CYCLES) % 2 == 0;
          intensity = flashOn ? UIConstants.RUMBLE_TRANSITION_INTENSITY : 0.0;
          break;

        case GREEN_SOLID:
        case RED_SOLID:
        default:
          // No rumble during solid states
          intensity = 0.0;
          break;
      }
    }

    setRumble(intensity);
  }

  /**
   * Sets rumble intensity on both controllers.
   *
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  private void setRumble(double intensity) {
    driverController.setRumble(RumbleType.kBothRumble, intensity);
    operatorController.setRumble(RumbleType.kBothRumble, intensity);
  }

  /**
   * Manually triggers a strong rumble for the specified controller. Used for button-triggered
   * feedback.
   *
   * @param controller The controller to rumble
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  public void manualRumble(GenericHID controller, double intensity) {
    controller.setRumble(RumbleType.kBothRumble, intensity);
  }

  /**
   * Stops manual rumble for the specified controller.
   *
   * @param controller The controller to stop rumbling
   */
  public void stopManualRumble(GenericHID controller) {
    controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  /**
   * Creates a command that rumbles the specified controller at full intensity while held.
   *
   * @param controller The controller to rumble
   * @return Command that rumbles on initialize and stops on end
   */
  public Command manualRumbleCommand(GenericHID controller) {
    return runEnd(
        () -> manualRumble(controller, UIConstants.RUMBLE_MANUAL_INTENSITY),
        () -> stopManualRumble(controller));
  }

  /**
   * Creates a command that rumbles the specified controller at 50% intensity for a fixed duration.
   * This overrides automatic rumble patterns during execution.
   *
   * @param controller The controller to rumble
   * @param seconds Duration in seconds
   * @return Command that rumbles for specified duration then stops
   */
  public Command timedRumbleCommand(GenericHID controller, double seconds) {
    return Commands.sequence(
        runOnce(
            () -> {
              manualOverride = true;
              controller.setRumble(RumbleType.kBothRumble, 0.5);
            }),
        Commands.waitSeconds(seconds),
        runOnce(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0.0);
              manualOverride = false;
            }));
  }
}
