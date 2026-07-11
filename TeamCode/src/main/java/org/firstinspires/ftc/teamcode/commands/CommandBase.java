// In package: org.firstinspires.ftc.teamcode.commands
package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Blackboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.Locale;

/**
 * An abstract base class for commands that provides a default implementation for
 * the transition condition logic.
 */
public abstract class CommandBase implements Command {
    protected TransitionCondition transitionCondition = null;
    private String commandName;
    private boolean hasTriggeredTransition = false;

    public static void log(String commandName, String message) {
        System.out.printf(Locale.US, "[%.3fs] COMMAND %s: %s%n",
                CommandScheduler.getElapsedTime(),
                commandName,
                message);
    }

    public static void logError(String commandName, String message) {
        System.err.printf(Locale.US, "[%.3fs] ERROR COMMAND %s: %s%n",
                CommandScheduler.getElapsedTime(),
                commandName,
                message);
    }

    /**
     * Save the final state of the robot to the OpMode blackboard.
     * This should be called from the stop() method of an OpMode.
     * @param drivetrain The Drivetrain subsystem.
//     * @param turret The Turret subsystem.
     */
    public static void saveAutoState(Drivetrain drivetrain) {
        if (drivetrain != null) {
            Pose2D finalPose = drivetrain.getPose();
//            double finalTurretAngle = turret.getCurrentAngle();
            Alliance.Color alliance = Alliance.Color.BLUE; //Add Auto Detect

            OpMode.blackboard.put(Blackboard.ROBOT_POSE_KEY, finalPose);
//            OpMode.blackboard.put(Blackboard.TURRET_ANGLE_KEY, finalTurretAngle);
            OpMode.blackboard.put(Blackboard.ALLIANCE_KEY, alliance);

            System.out.printf(Locale.US, "AUTO END: Saving state. Alliance: %s, Pose: (%.1f, %.1f, %.1f)%n",
                    alliance != null ? alliance.toString() : "UNKNOWN",
                    finalPose.getX(DistanceUnit.INCH),
                    finalPose.getY(DistanceUnit.INCH),
                    finalPose.getHeading(AngleUnit.DEGREES));
        } else {
            System.err.println("AUTO END: Could not save state! A required object was null.");
        }
    }

    public CommandBase() {
        // Default name is the simple name of the class (e.g., "FollowPathCommand")
        this.commandName = this.getClass().getSimpleName();
    }

    /**
     * Sets a custom name for this command instance for better logging.
     * @param name The custom name for this command.
     * @return The command instance, for method chaining.
     */
    public CommandBase withName(String name) {
        this.commandName = name;
        return this;
    }

    @Override
    public String getName() {
        return this.commandName;
    }

    @Override
    public void setTransitionCondition(TransitionCondition condition) {
        this.transitionCondition = condition;
    }

    @Override
    public boolean isReadyForNext(Drivetrain drivetrain) {
        // If this command has already triggered a transition, it can't trigger another one.
        if (hasTriggeredTransition) {
            return false;
        }

        boolean isReady = false;
        // If a custom condition is set, use it.
        if (transitionCondition != null) {
            isReady = transitionCondition.shouldTransition(drivetrain);
        } else {
            // Default behavior: ready for next only when fully finished.
            isReady = isFinished();
        }

        // If the command is ready to transition, set the flag so it doesn't trigger again.
        if (isReady) {
            this.hasTriggeredTransition = true;
            return true;
        }

        return false;

//        // If a custom condition is set, use it. Otherwise, default to finishing.
//        if (transitionCondition != null) {
//            return transitionCondition.shouldTransition(drivetrain, intake, shooter, turret);
//        }
//        // Default behavior: ready for next only when fully finished.
//        return isFinished();
    }

    /**
     * Resets the transition flag. This is called by the scheduler when the command starts
     * to ensure it can trigger a transition if it's reused.
     */
    public void resetTransition() {
        this.hasTriggeredTransition = false;
    }

    // Abstract methods from the Command interface that concrete classes must implement
    @Override
    public abstract void start(Drivetrain drivetrain);
    @Override
    public abstract void update();
    @Override
    public abstract boolean isFinished();
    @Override
    public abstract void end();
}
