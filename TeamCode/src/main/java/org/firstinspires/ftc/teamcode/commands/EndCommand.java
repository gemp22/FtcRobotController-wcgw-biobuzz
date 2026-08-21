package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * A command that stops all robot subsystems after a brief delay.
 * This is intended to be the final command in an autonomous sequence
 * to ensure the robot is stationary and all mechanisms are off,
 * allowing time for final actions like shooting to complete.
 */
public class EndCommand extends CommandBase {

    private ElapsedTime delayTimer;
    private static final double SHUTDOWN_DELAY_SECONDS = 3.0;

    private Drivetrain drivetrain;
//    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;

    /**
     * Override withName to allow for proper method chaining.
     * @param name The custom name for this command.
     * @return The command instance.
     */
    @Override
    public EndCommand withName(String name) {
        super.withName(name);
        return this;
    }

    /**
     * When the command starts, it initializes and starts a timer.
     * The actual stopping action is moved to the update() and isFinished() logic.
     */
    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
        log(getName(), String.format("Started. Shutdown sequence with a %.1f-second delay.", SHUTDOWN_DELAY_SECONDS));

        this.drivetrain = drivetrain;
//        this.intake = intake;
//        this.shooter = shooter;
//        this.turret = turret;

        if (delayTimer == null) {
            delayTimer = new ElapsedTime();
        }
        delayTimer.reset();
    }

    /**
     * This command has no ongoing actions, so this method is empty.
     */
    @Override
    public void update() {
        // Nothing to do. The action happens instantly in start().
    }

    /**
     * This command is finished once the timer has exceeded the specified delay.
     * @return True if the delay has passed, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return delayTimer != null && delayTimer.seconds() >= SHUTDOWN_DELAY_SECONDS;
    }

    /**
     * Called once when isFinished() returns true.
     * This is where the subsystems will now be stopped.
     */
    @Override
    public void end() {
        log(getName(), "Delay finished. Stopping all subsystems.");

        if (drivetrain != null) {
            drivetrain.stop();
        }
//        if (intake != null) {
//            intake.stop();
//        }
//        if (shooter != null) {
//            shooter.stop();
//        }
//        if (turret != null) {
//            turret.stop();
//        }
        log(getName(), "Finished.");
    }
}
