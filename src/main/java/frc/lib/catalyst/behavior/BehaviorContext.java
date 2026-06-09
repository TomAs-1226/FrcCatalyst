package frc.lib.catalyst.behavior;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.catalyst.util.RobotState;

/**
 * Lightweight snapshot of "what's the situation right now" handed to
 * {@link Strategist} scorers and used internally by {@link BehaviorEngine}.
 *
 * <p>Deliberately small and game-agnostic: it exposes match-phase and
 * timing only. Anything game-specific (do we have a piece? how many
 * scored?) is read by the scorer's own lambdas closing over the team's
 * subsystems — the framework never needs to know what a "piece" is.
 */
public final class BehaviorContext {

    private final double startTime;

    BehaviorContext(double startTime) {
        this.startTime = startTime;
    }

    /** Seconds left in the current match phase (auto or teleop), per the FMS / DS. */
    public double matchTimeRemaining() {
        return RobotState.matchTimeRemaining();
    }

    /** Seconds since this behavior started running. */
    public double elapsed() {
        return Timer.getFPGATimestamp() - startTime;
    }

    public boolean isAutonomous() {
        return RobotState.isAutonomous();
    }

    public boolean isTeleop() {
        return RobotState.isTeleop();
    }

    public boolean isRed() {
        return RobotState.isRed();
    }

    public double batteryVoltage() {
        return RobotState.batteryVoltage();
    }
}
