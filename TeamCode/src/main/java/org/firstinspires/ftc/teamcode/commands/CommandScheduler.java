// In package: org.firstinspires.ftc.teamcode.commands
package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class CommandScheduler {
    private static final ElapsedTime masterTimer = new ElapsedTime();
    private static boolean isTimerStarted = false;

    // Deadline configuration
    private final double autoDeadlineS;
    private boolean isTeardownActive = false;

    private final Drivetrain drivetrain;
    private final Intake intake;
//    private final Shooter shooter;
//    private final Turret turret;

    private final Queue<Command> mainQueue = new LinkedList<>();
    private final Queue<Command> teardownQueue = new LinkedList<>();
    private final List<Command> activeCommands = new ArrayList<>();

    private boolean hasSequenceStarted = false;

    private boolean useAutoDeadline = true; // Enable/disable deadline logic


    /**
     * Constructor with a configurable deadline
     * @param drivetrain The robot's Drivetrain subsystem.
     * @param intake The robot's Intake subsystem.
//     * @param shooter The robot's Shooter subsystem.
//     * @param turret The robot's Turret subsystem.
     * @param deadlineSeconds The time in seconds at which to force the teardown sequence.
     */
    public CommandScheduler(Drivetrain drivetrain, Intake intake, double deadlineSeconds) {
        this.drivetrain = drivetrain;
        this.intake = intake;
//        this.shooter = shooter;
//        this.turret = turret;
        this.autoDeadlineS = deadlineSeconds; // Set the deadline from the parameter
        isTimerStarted = false;
        masterTimer.reset();
    }

    /**
     * Creates a CommandScheduler with a default teardown deadline of 28.0 seconds.
     * @param drivetrain The Drivetrain subsystem.
     * @param intake The Intake subsystem.
//     * @param shooter The Shooter subsystem.
//     * @param turret The Turret subsystem.
     */
    public CommandScheduler(Drivetrain drivetrain, Intake intake) {
        // Chain to the new constructor, providing a default value.
        this(drivetrain, intake, 28.0);
    }

    public static double getElapsedTime() {
        return masterTimer.seconds();
    }

    /**
     * Forcibly stops all active commands and clears the main execution queue.
     * This is useful for driver overrides in TeleOp.
     * Note: This does NOT clear the teardown queue or reset the match timer.
     */
    public void cancelAll() {
        if (!activeCommands.isEmpty() || !mainQueue.isEmpty()) {
            CommandBase.log("SCHEDULER", "Cancelling all active and queued commands.");
        }

        // 1. Clear the main queue so no more commands start automatically
        mainQueue.clear();

        // 2. End and remove all currently running commands
        for (Command cmd : new ArrayList<>(activeCommands)) {
            cmd.end();
        }
        activeCommands.clear();

        // Reset these flags so the scheduler doesn't think it's
        // in the middle of an autonomous sequence anymore.
        this.hasSequenceStarted = false;
        this.isTeardownActive = false;
    }

    /**
     * Disables the 28-second deadline and automatic teardown transition.
     * Call this in TeleOp OpModes.
     */
    public void setTeleOpMode(boolean isTeleOp) {
        this.useAutoDeadline = !isTeleOp;
    }

    /**
     * Adds a command to the main execution queue.
     * @param command The command to add.
     */
    public void add(Command command) {
        mainQueue.add(command);
    }

    /**
     * Adds a command to the mandatory teardown sequence.
     * These commands will run after the deadline is hit or the main sequence finishes.
     * @param command The teardown command to add.
     */
    public void addTeardown(Command command) {
        teardownQueue.add(command);
    }

    /**
     * The main update loop for the scheduler. This should be called in your OpMode's loop().
     */
    public void update() {
        // Only check deadline if we are NOT in TeleOp mode
        if (useAutoDeadline && !isTeardownActive && isTimerStarted && getElapsedTime() > autoDeadlineS) {
            CommandBase.logError("SCHEDULER", String.format("AUTO DEADLINE REACHED! Forcing teardown.", autoDeadlineS));
            forceTeardown();
        }

        // 1. Update all currently active commands and remove any that are finished.
        // We use a copy to avoid modification issues while iterating.
        for (Command cmd : new ArrayList<>(activeCommands)) {
            cmd.update();
            if (cmd.isFinished()) {
                CommandBase.log("SCHEDULER", String.format("Command '%s' finished. Ending and removing.", cmd.getName()));
                cmd.end();
                activeCommands.remove(cmd);
            }
        }

        // 2. If no commands are active, try to start the next one.
        if (activeCommands.isEmpty()) {
            // Prioritize the teardown queue if it's active
            if (isTeardownActive && !teardownQueue.isEmpty()) {
                startNextCommand(teardownQueue);
                return;
            }
            // Otherwise, run from the main queue
            if (!isTeardownActive && !mainQueue.isEmpty()) {
                startNextCommand(mainQueue);
                return;
            }
        }

        // 3. Handle transitions for parallel commands.
        Queue<Command> queueToCheck = isTeardownActive ? teardownQueue : mainQueue;
        if (!queueToCheck.isEmpty()) {
            for (Command cmd : activeCommands) {
                if (cmd.isReadyForNext(drivetrain, intake)) {
                    CommandBase.log("SCHEDULER", String.format("Command '%s' is ready for transition. Starting next.", cmd.getName()));
                    startNextCommand(queueToCheck);
                    break;
                }
            }
        }

        // 4. If everything is done (main queue, active commands), start the teardown sequence naturally.
        // Only transition to teardown automatically if NOT in TeleOp mode
        if (useAutoDeadline && !isBusy() && !isTeardownActive && hasSequenceStarted) {
            CommandBase.log("SCHEDULER", "Main sequence finished. Starting teardown sequence.");
            isTeardownActive = true;
        }

//        // If the scheduler is no longer busy, but the sequence had started, it means we just finished.
//        if (!isBusy() && hasSequenceStarted) {
//            System.out.println("#################### END COMMANDS ####################");
//            hasSequenceStarted = false; // Reset for the next run
//        }
    }

    /**
     * Forcibly stops all current and queued main commands and activates the teardown sequence.
     */
    private void forceTeardown() {
        // End all currently running commands
        for (Command cmd : activeCommands) {
            CommandBase.logError("SCHEDULER", String.format("DEADLINE: Forcibly ending active command '%s'.", cmd.getName()));
            cmd.end();
        }
        activeCommands.clear();

        // Clear any remaining commands in the main queue
        if (!mainQueue.isEmpty()) {
            CommandBase.logError("SCHEDULER", String.format("DEADLINE: Clearing %d remaining commands from main queue.", mainQueue.size()));
            mainQueue.clear();
        }

        // Activate teardown mode
        isTeardownActive = true;
    }

    /**
     * Starts the next command from the specified queue.
     * @param commandQueue The queue (main or teardown) to pull from.
     */
    private void startNextCommand(Queue<Command> commandQueue) {
        if (!commandQueue.isEmpty()) {
            if (!hasSequenceStarted) {
                System.out.println("\n################### START COMMANDS ###################");
                hasSequenceStarted = true;

                if (!isTimerStarted) {
                    masterTimer.reset();
                    isTimerStarted = true;
                }
            }

            Command nextCmd = commandQueue.poll(); // Retrieves and removes the head of the queue
            if (nextCmd != null) {
                CommandBase.log("SCHEDULER", String.format("Pulling from queue and starting: '%s'", nextCmd.getName()));
                // Reset the transition flag before starting, in case the command is being reused.
                nextCmd.resetTransition();
                nextCmd.start(drivetrain, intake);
                activeCommands.add(nextCmd);
            }
        }
    }

    /**
     * Checks if all commands have been completed.
     * @return True if the queue and active list are both empty.
     */
    public boolean isBusy() {
        // The scheduler is busy if any command is active or any queue has commands.
        return !activeCommands.isEmpty() || !mainQueue.isEmpty() || !teardownQueue.isEmpty();
    }

    /**
     * Clears the command queue and stops all active commands.
     */
    public void reset() {
        CommandBase.log("SCHEDULER", "Resetting. Clearing all queues and ending active commands.");
        mainQueue.clear();
        teardownQueue.clear();
        for (Command cmd : activeCommands) {
            cmd.end();
        }
        activeCommands.clear();
        isTimerStarted = false;
        hasSequenceStarted = false;
        isTeardownActive = false; // Also reset the teardown flag
    }

    /**
     * Gets the name of the current "main" command for telemetry.
     * The main command is the last one that was added to the active list.
     * @return The name of the current command, or null if no commands are active.
     */
    public String getCurrentCommandName() {
        if (!activeCommands.isEmpty()) {
            // Return the name of the last command in the active list, as it's the one
            // the scheduler is waiting on for transitions.
            return activeCommands.get(activeCommands.size() - 1).getName();
        }
        return null; // No active commands
    }
}
