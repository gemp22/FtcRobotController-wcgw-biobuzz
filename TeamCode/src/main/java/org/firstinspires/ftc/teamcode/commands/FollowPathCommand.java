package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;

/**
 * A command that instructs the Drivetrain subsystem to follow a pre-defined path using its
 * pure pursuit algorithm, with a timeout for safety.
 */
public class FollowPathCommand extends CommandBase {

    // The subsystem this command requires
    private Drivetrain drivetrain;

    // The data this command needs to execute
    private final ArrayList<CurvePoint> pathToFollow;
    private final double followAngle;
    private final boolean debug;
    private final double timeoutSeconds;

    // Timer for the timeout
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Main constructor for the FollowPathCommand.
     * @param path The list of CurvePoints that defines the path for the robot to follow.
     * @param followAngle The angle (in radians) the robot should maintain relative to the path.
     * @param debug If true, sends debug data to the field simulator for this path.
     * @param timeoutInSeconds The maximum time in seconds this command is allowed to run.
     */
    public FollowPathCommand(ArrayList<CurvePoint> path, double followAngle, boolean debug, double timeoutInSeconds) {
        this.pathToFollow = path;
        this.followAngle = followAngle;
        this.debug = debug;
        this.timeoutSeconds = timeoutInSeconds;
    }

    /**
     * Convenience constructor with a default timeout of 10 seconds.
     */
    public FollowPathCommand(ArrayList<CurvePoint> path, double followAngle, boolean debug) {
        this(path, followAngle, debug, 10.0); // Default to 10-second timeout
    }

    /**
     * A convenience constructor that defaults debugging to false and timeout to 10 seconds.
     */
    public FollowPathCommand(ArrayList<CurvePoint> path, double followAngle) {
        this(path, followAngle, false, 10.0); // Defaults to not debugging and 10s timeout
    }

    /**
     * Sets a transition condition to start the next command when the robot is within a specified
     * distance of the final point of this command's path.
     * @param distanceInches The distance threshold in inches.
     * @return The command instance, allowing for method chaining.
     */
    public FollowPathCommand transitionWhenDistancetoEndIsLessThan(double distanceInches) {
        // Ensure the path is not empty to avoid errors
        if (this.pathToFollow == null || this.pathToFollow.isEmpty()) {
            logError(getName(), "ERROR: Cannot set transition condition on a null or empty path.");
            return this;
        }

        // Get the final point of the path
        CurvePoint finalPoint = this.pathToFollow.get(this.pathToFollow.size() - 1);

        // Create a lambda that implements the TransitionCondition interface.
        // This lambda captures the finalPoint and distanceInches values.
        TransitionCondition condition = (d, i) -> {
            // Get the robot's current pose from the Drivetrain subsystem
            double robotX = d.getPose().getX(DistanceUnit.INCH);
            double robotY = d.getPose().getY(DistanceUnit.INCH);

            // Calculate the distance to the final point
            double distanceToEnd = Math.hypot(finalPoint.x - robotX, finalPoint.y - robotY);

            // The condition is met if the distance is less than the specified threshold
            return distanceToEnd < distanceInches;
        };

        // Set the condition for this command instance
        this.setTransitionCondition(condition);

        // Return "this" to allow for chaining, e.g., new FollowPathCommand(...).transitionWhen(...)
        return this;
    }

    /**
     * Sets a transition condition that allows the next command to start immediately.
     * This enables this command to run in parallel with subsequent commands, which is useful
     * when the path is very short and you need the next command (like a wait) to start right away.
     * @return The command instance, for method chaining.
     */
    public FollowPathCommand transitionImmediately() {
        // A lambda that always returns true, so the scheduler moves on right away.
        this.setTransitionCondition((d, i) -> true);
        return this;
    }

    @Override
    public FollowPathCommand withName(String name) {
        super.withName(name); // Call the parent method to set the name
        return this;          // Return this specific type to allow chaining
    }

    /**
     * Called when the command begins. It tells the Drivetrain to start following the path.
     */
    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
        this.drivetrain = drivetrain;
        timer.reset(); // Start the timeout timer
        log(getName(), String.format("Started (Timeout: %.1fs).", timeoutSeconds));
        // Tell the drivetrain to start its action
        this.drivetrain.followPath(this.pathToFollow, this.followAngle, this.debug);
    }

    /**
     * Called repeatedly. For this command, we don't need to do anything here because the
     * Drivetrain's own update() method is already handling the path following logic in the background.
     */
    @Override
    public void update() {
        // Nothing to do here. The subsystem is doing the work.
    }

    /**
     * This command is finished when the Drivetrain subsystem is no longer busy OR the timeout is reached.
     * @return True when the path is complete or timed out, false otherwise.
     */
    @Override
    public boolean isFinished() {
        if (drivetrain == null) {
            return true; // Failsafe
        }
        // The command is finished if the drivetrain is no longer busy OR if the timer has exceeded the timeout.
        return !drivetrain.isBusy() || timer.seconds() > timeoutSeconds;
    }

    /**
     * Called when the command ends. Logs whether the command finished normally or timed out.
     */
    @Override
    public void end() {
        // Check if the command ended because of the timeout
        if (timer.seconds() > timeoutSeconds) {
            logError(getName(), String.format("TIMED OUT after %.1fs!", timeoutSeconds));
            // It's good practice to force the drivetrain to stop if it times out
            if (drivetrain != null) {
                drivetrain.stop();
            }
        } else {
            log(getName(), "Finished normally.");
        }
    }
}
