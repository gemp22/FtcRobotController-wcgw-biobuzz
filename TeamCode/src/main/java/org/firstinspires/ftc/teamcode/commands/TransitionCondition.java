// In package: org.firstinspires.ftc.teamcode.commands
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * A functional interface for defining custom conditions under which a command
 * can transition to the next one in the sequence, allowing for command overlap.
 */
@FunctionalInterface
public interface TransitionCondition {
    /**
     * Checks if the condition to transition to the next command is met.
     *
     * @param drivetrain The robot's Drivetrain subsystem.
//     * @param intake The robot's Intake subsystem.
//     * @param shooter The robot's Shooter subsystem.
//     * @param turret The robot's Turret subsystem.
     * @return True if the next command should be started, false otherwise.
     */
    boolean shouldTransition(Drivetrain drivetrain);
}
