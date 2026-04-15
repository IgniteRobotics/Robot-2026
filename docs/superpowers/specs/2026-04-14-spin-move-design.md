# Spin Move Design

**Date:** 2026-04-14
**Branch:** feature/spin-move
**File:** `src/main/java/frc/robot/subsystems/drive/DrivetrainSubsystem.java`

## Problem

The existing `spinMoveCommand` uses a fixed field-coordinate pivot and sets the swerve center of rotation to that point each tick. A full 360° orbit around a fixed field point returns the robot to its starting position — making it impossible to achieve both automatic lateral displacement and a full rotation simultaneously. The rotation direction also incorrectly flips based on `Math.signum(vx)`, violating the requirement that left trigger always spins the same direction regardless of travel direction.

## Coordinate System

- Origin (0, 0): bottom-right corner of the blue alliance side of the field
- +X: toward red alliance end
- +Y: lateral (toward the other side of the field)
- Blue driver faces +X. Their left is +Y.
- Red driver faces -X. Their left is -Y.

## Requirements

- Left trigger → robot moves to driver's left; right trigger → driver's right. Always field-relative, alliance-aware.
- Rotation direction is fixed per trigger per alliance — does not flip based on travel direction.
- Automatic lateral displacement of `1 robot width + 2 × offset` over one full rotation. This is automatic — not driven by the driver's Y input.
- Driver vx (forward/backward) is fully honored during the spin.
- Driver vy is added on top of the automatic lateral velocity.
- Command ends when: heading delta ≥ 2π AND |field Y displacement| ≥ target lateral distance.

## Design

### Key Values

```
pivotDist     = robotCenterToEdge + SPIN_MOVE_PIVOT_CLEARANCE
              = 0.47m + 0.05m = 0.52m

targetLateral = 2 × pivotDist = 1.04m

ω             = spinMoveAngularSpeed (DrivePreferences, default = MAX_ANGULAR_SPEED)

auto_vy       = targetLateral × ω / (2π)
              ≈ 1.04 × 4.71 / 6.28 ≈ 0.78 m/s at default speed
```

`auto_vy` is derived from `targetLateral` and `ω` so that after exactly one full rotation the robot has automatically displaced one full lane laterally. If the driver adds Y input on top, they reach the target sooner but the command still waits for full rotation.

### Direction Signs

```
allianceSign = +1 (blue) / -1 (red)
lateralDir   = (pivotLeft ? +1 : -1) × allianceSign
omegaSign    = (pivotLeft ? -1 : +1) × allianceSign
```

Blue + left trigger: lateralDir = +1 (+Y = driver's left ✓), omegaSign = -1 (CW ✓).
Red + left trigger: lateralDir = -1 (-Y = driver's left ✓), omegaSign = +1 (CW from red driver's perspective ✓).

### Per-Tick Swerve Request

```
vx             = driver vx  (field-centric)
vy             = (auto_vy × lateralDir) + driver vy
rotationalRate = omegaSign × spinMoveAngularSpeed
```

No center-of-rotation offset. No orbit math.

### Initialization (runOnce)

- Record `startY` = current field Y position
- Record `startHeading` = current heading
- Reset `headingDelta` = 0.0

### Termination

```
|headingDelta| ≥ 2π  AND  |currentY − startY| ≥ targetLateral
```

The first condition ensures a full rotation. The second ensures the lane change is complete. Both should be satisfied simultaneously under nominal conditions (auto_vy is calibrated for this). The command also terminates early if the driver releases the trigger (`.whileTrue()` in RobotContainer) or if the 3-second timeout fires.

## What Changes

### `DrivetrainSubsystem.java` — `spinMoveCommand`

Full rewrite of the method body:

- Remove: fixed field pivot, `state.fieldPivot`, `state.omegaSign` via `signum(vx)`, robot-relative pivot recalculation, validity check (`SPIN_MOVE_MIN_SPEED_MPS`, direction check).
- Add: `startY`, `auto_vy`, `lateralDir`, `omegaSign` (no vx dependency), heading accumulation, lateral displacement termination check.

### `DriveConstants.java`

- `SPIN_MOVE_MIN_SPEED_MPS` can be removed (validity check eliminated).
- `SPIN_MOVE_PIVOT_CLEARANCE_METERS` stays (used to compute `pivotDist` and `targetLateral`).

### `DrivePreferences.java`

No changes. `spinMoveAngularSpeed` stays and is used to derive `auto_vy`.

### `RobotContainer.java`

No changes. Trigger bindings and timeout are already correct.
