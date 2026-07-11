package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * A simple command that does nothing but wait for a specified amount of time.
 * This is useful for adding delays in an autonomous sequence.
 */
public class WaitCommand extends CommandBase {

    private final double timeoutSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Creates a new WaitCommand.
     * @param seconds The number of seconds to wait before the command finishes.
     */
    public WaitCommand(double seconds) {
        this.timeoutSeconds = seconds;
    }

    /**
     * When the command starts, it resets the timer.
     */
    @Override
    public void start(Drivetrain drivetrain) {
        timer.reset();
        log(getName(), String.format("Waiting for %.2f seconds.", timeoutSeconds));
    }

    /**
     * The update loop is empty, as this command performs no continuous action.
     */
    @Override
    public void update() {
        // Nothing to do here.
    }

    /**
     * The command is finished once the timer has exceeded the specified timeout.
     * @return True if the wait time has elapsed, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return timer.seconds() >= timeoutSeconds;
    }

    /**
     * The end method is empty, as there is no state to clean up.
     */
    @Override
    public void end() {
        // Nothing to clean up.
    }
}
