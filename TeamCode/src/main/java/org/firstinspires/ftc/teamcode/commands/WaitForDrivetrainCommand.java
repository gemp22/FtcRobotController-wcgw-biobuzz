// Create this new file in: org.firstinspires.ftc.teamcode.commands
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * A simple command that waits until the Drivetrain is no longer busy.
 * This is useful for synchronizing other actions to the completion of a path.
 */
public class WaitForDrivetrainCommand extends CommandBase {

    private Drivetrain drivetrain;

    @Override
    public void start(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        log(getName(), "Now waiting for drivetrain to finish its path.");
    }

    @Override
    public void update() {
        // Nothing to do here, we are just waiting.
    }

    @Override
    public boolean isFinished() {
        if (drivetrain == null) {
            return true; // Failsafe
        }
        // This command is finished when the drivetrain is no longer busy.
        return !drivetrain.isBusy();
    }

    @Override
    public void end() {
        log(getName(), "Drivetrain has arrived. Proceeding.");
    }
}
