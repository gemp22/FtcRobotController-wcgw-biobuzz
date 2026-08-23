package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Locale;

/**
 * A command to set the robot's initial position and heading in the odometry system.
 * This command finishes immediately.
 */
public class SetInitialPoseCommand extends CommandBase {

    private final Pose2D initialPose;

    /**
     * Creates a new SetInitialPoseCommand.
     * @param initialPose The pose to set the robot to.
     */
    public SetInitialPoseCommand(Pose2D initialPose) {
        this.initialPose = initialPose;
    }

    /**
     * Creates a new SetInitialPoseCommand using inches and degrees.
     * @param x The X position in inches.
     * @param y The Y position in inches.
     * @param heading The heading in degrees.
     */
    public SetInitialPoseCommand(double x, double y, double heading) {
        this(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
        if (drivetrain != null) {
            log(getName(), String.format(Locale.US, "Setting initial pose to: X:%.2f, Y:%.2f, H:%.2f",
                    initialPose.getX(DistanceUnit.INCH),
                    initialPose.getY(DistanceUnit.INCH),
                    initialPose.getHeading(AngleUnit.DEGREES)));
            drivetrain.setPosition(initialPose);
        } else {
            logError(getName(), "Drivetrain subsystem is null! Cannot set position.");
        }
    }

    @Override
    public void update() {
        // Nothing to do.
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end() {
        // No cleanup required.
    }
}
