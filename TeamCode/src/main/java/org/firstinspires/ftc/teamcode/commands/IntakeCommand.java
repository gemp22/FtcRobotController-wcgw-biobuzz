package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * A command that allows for controlling the state of the Intake subsystem.
 * This command sets the intake to a specified state (FORWARD, REVERSE, or OFF)
 * and finishes immediately.
 */
public class IntakeCommand extends CommandBase {

    private final Intake.IntakeState targetState;

    /**
     * Creates a new IntakeCommand to set the intake state.
     * @param targetState The desired state for the intake.
     */
    public IntakeCommand(Intake.IntakeState targetState) {
        this.targetState = targetState;
    }

    /**
     * Called when the command begins. It instructs the Intake subsystem to change its state.
     */
    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
        if (intake != null) {
            log(getName(), "Setting intake state to: " + targetState);
            intake.changeState(targetState);
        } else {
            logError(getName(), "Intake subsystem is null! Cannot change state.");
        }
    }

    /**
     * Called repeatedly while the command is running.
     * Setting the state is instantaneous, so no continuous logic is needed here.
     */
    @Override
    public void update() {
        // Nothing to do.
    }

    /**
     * This command finishes immediately after requesting the state change in start().
     * @return Always true.
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Called when the command ends.
     */
    @Override
    public void end() {
        // No cleanup required.
    }
}
