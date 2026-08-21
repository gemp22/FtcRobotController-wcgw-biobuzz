package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * An interface that defines a standard structure for an autonomous command.
 * Each command represents a single, specific action for the robot to perform.
 */
public interface Command {

    /**
     * Gets the name of the command for logging purposes.
     * @return The name of the command.
     */
    String getName();

    /**
     * Called once when the command is first scheduled to run.
     * This is where you would tell a subsystem to start an action.
     * @param drivetrain The robot's Drivetrain subsystem.
     * @param intake The robot's Intake subsystem.
//     * @param shooter The robot's Shooter subsystem.
//     * @param turret The robot's Turret subsystem.
     */
    void start(Drivetrain drivetrain, Intake intake);

    /**
     * Called repeatedly in the main OpMode loop while the command is running.
     * For most commands, this will be empty, as the subsystem's own update()
     * method handles the continuous logic.
     */
    void update();

    /**
     * Determines if the command has finished its action.
     * @return True if the command is finished, false otherwise.
     */
    boolean isFinished();

    /**
     * Sets a custom condition that determines when the scheduler can move to the next command.
     * @param condition The TransitionCondition to evaluate.
     */
    void setTransitionCondition(TransitionCondition condition);

    /**
     * Resets any internal state related to transitions, allowing the command to be reused.
     */
    void resetTransition();

    /**
     * Determines if the command is ready for the next command to start.
     * By default, this is the same as isFinished(), but can be overridden by a TransitionCondition.
     * @param drivetrain The robot's Drivetrain subsystem.
     * @param intake The robot's Intake subsystem.
//     * @param shooter The robot's Shooter subsystem.
//     * @param turret The robot's Turret subsystem.
     * @return True if the next command can be started, false otherwise.
     */
    boolean isReadyForNext(Drivetrain drivetrain, Intake intake);

    /**
     * Called once when the command finishes (i.e., when isFinished() returns true).
     * This is the place to clean up, such as telling a subsystem to stop.
     */
    void end();
}
