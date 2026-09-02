package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;


import java.util.Locale;

/**
 * Autonomous command to locate and intake balls using the intake camera.
 * Prioritizes clusters and uses PID-controlled rotation to point the robot at targets.
 */
public class AutoIntakeCommand extends CommandBase {

    private enum State {
        ROTATING_TO_START, // New: Rotate to initial heading
        SEARCHING,   // Waiting for a ball to appear
        SCANNING,    // Rotating slowly to find a ball
        CHASING,     // Moving toward the ball using PID rotation
        BLIND_FINISH, // Driving the last bit after the ball is too close to see
        DONE
    }

    // --- Configuration Constants ---
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;
    private static final double INITIAL_ROTATION_SPEED_MAX = 0.4;
    private static final double INITIAL_ROTATION_KP = 0.1; // Proportional gain for heading

    private static final double SCAN_ROTATION_SPEED = 0.25;
    private static final double SCAN_RANGE_DEGREES = 15.0;
    private static final double CHASE_DRIVE_SPEED = 1.00;
    private static final int CAMERA_CENTER_X = 160; // HuskyLens 320x240

    // PID for rotation (pointing at the ball)
    private static final double KP_ROTATION = 0.005;
    private static final double MAX_ROTATION_POWER = 0.4;

    // --- State Variables ---
    private State currentState = State.ROTATING_TO_START;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime globalTimer = new ElapsedTime();

    private Pose2D startPose;
    private double scanStartHeading;
    private boolean isScanningClockwise = true;

    // --- User Arguments ---
    private final double targetInitialHeading;
    private final boolean canScan;
    private final double searchTimeout;
    private final double blindDriveInches;
    private final double maxTotalDistance;

    private Drivetrain drivetrain;
    private Intake intake;

    // --- Perimeter Constants ---
    private static final double PERIM_MIN_X = -66.0;
    private static final double PERIM_MAX_X = -6.0;
    private static final double PERIM_BLUE_MIN_Y = 6.0;
    private static final double PERIM_BLUE_MAX_Y = 55.0;
    private static final double PERIM_RED_MIN_Y = -55.0;
    private static final double PERIM_RED_MAX_Y = -6.0;

    private Alliance.Color alliance = Alliance.Color.BLUE; // Default to BLUE


    /**
     * @param targetInitialHeading The heading the robot should reach before starting to look for balls.
     * @param canScan If true, the robot will sweep left/right if no ball is initially seen.
     * @param searchTimeout How long to wait/scan before giving up (seconds).
     * @param blindDriveInches How much further to drive once the ball is lost under the camera (inches).
     * @param maxTotalDistance Safety limit for total travel (inches).
     */
    public AutoIntakeCommand(double targetInitialHeading, boolean canScan, double searchTimeout, double blindDriveInches, double maxTotalDistance) {
        this.targetInitialHeading = targetInitialHeading;
        this.canScan = canScan;
        this.searchTimeout = searchTimeout;
        this.blindDriveInches = blindDriveInches;
        this.maxTotalDistance = maxTotalDistance;
    }

    public AutoIntakeCommand(double targetInitialHeading) {
        this(targetInitialHeading, true, 3.0, 8.0, 60.0); // Default values
    }

    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.alliance = Alliance.Color.RED;

        intake.enableCameraSensing(true);

        this.startPose = drivetrain.getPose();
        this.currentState = State.ROTATING_TO_START;

        stateTimer.reset();
        globalTimer.reset();

        log(getName(), String.format(Locale.US, "Started Auto Intake. Initial Heading Target: %.1f", targetInitialHeading));
    }

    @Override
    public void update() {
        // Safety check for safe perimeter
        if (!isInsideSafePerimeter()) {
            Pose2D p = drivetrain.getPose();
            logError(getName(), String.format(Locale.US, "PERIMETER BREACH! (X:%.1f, Y:%.1f). Terminating.", p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH)));
            currentState = State.DONE;
            return;
        }

        // Safety check for total distance moved
        if (getDistanceMoved() > maxTotalDistance) {
            logError(getName(), "Max distance reached! Terminating.");
            currentState = State.DONE;
            return;
        }

        switch (currentState) {
            case ROTATING_TO_START:
                double currentHeading = drivetrain.getPose().getHeading(AngleUnit.DEGREES);
                // Normalize error to [-180, 180] to find the shortest rotation path
                double headingError = AngleUnit.normalizeDegrees(targetInitialHeading - currentHeading);

                if (Math.abs(headingError) < HEADING_TOLERANCE_DEGREES) {
                    drivetrain.stop();
                    // Set the scan reference to the actual heading we reached
                    this.scanStartHeading = drivetrain.getPose().getHeading(AngleUnit.DEGREES);
                    transitionTo(State.SEARCHING);
                } else {
                    double rotPower = Range.clip(headingError * INITIAL_ROTATION_KP, -INITIAL_ROTATION_SPEED_MAX, INITIAL_ROTATION_SPEED_MAX);
                    drivetrain.drive(0, 0, rotPower);
                }
                break;

            case SEARCHING:
                if (intake.isBallInFront()) {
                    transitionTo(State.CHASING);
                } else if (stateTimer.seconds() > 0.5) {
                    if (canScan) transitionTo(State.SCANNING);
                    else if (stateTimer.seconds() > searchTimeout) transitionTo(State.DONE);
                }
                break;

            case SCANNING:
                if (intake.isBallInFront()) {
                    transitionTo(State.CHASING);
                    return;
                }

                if (stateTimer.seconds() > searchTimeout) {
                    log(getName(), "Scan timeout - no balls found.");
                    transitionTo(State.DONE);
                    return;
                }

                double scanHeading = drivetrain.getPose().getHeading(AngleUnit.DEGREES);
                double delta = AngleUnit.normalizeDegrees(scanHeading - scanStartHeading);

                if (isScanningClockwise && delta < -SCAN_RANGE_DEGREES) isScanningClockwise = false;
                else if (!isScanningClockwise && delta > SCAN_RANGE_DEGREES) isScanningClockwise = true;

                double scanRotPower = isScanningClockwise ? -SCAN_ROTATION_SPEED : SCAN_ROTATION_SPEED;
                drivetrain.drive(0, 0, scanRotPower);
                break;

            case CHASING:
                if (!intake.isBallInFront()) {
                    log(getName(), "Target lost close-up. Finishing blind.");
                    transitionTo(State.BLIND_FINISH);
                    return;
                }

                int targetX = intake.getFrontBallPositionX();
                double error = CAMERA_CENTER_X - targetX;
                double rx = Range.clip(error * KP_ROTATION, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
                drivetrain.drive(CHASE_DRIVE_SPEED, 0, rx);
                break;

            case BLIND_FINISH:
                drivetrain.drive(CHASE_DRIVE_SPEED, 0, 0);
                // Approximate distance using time/speed (30 inches per second estimate)
                if (stateTimer.seconds() * (CHASE_DRIVE_SPEED * 30) > blindDriveInches) {
                    transitionTo(State.DONE);
                }
                break;

            case DONE:
                drivetrain.stop();
                break;
        }
    }

    private boolean isInsideSafePerimeter() {
        Pose2D currentPose = drivetrain.getPose();
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);

        if (x < PERIM_MIN_X || x > PERIM_MAX_X) return false;

        if (alliance == Alliance.Color.BLUE) {
            return (y >= PERIM_BLUE_MIN_Y && y <= PERIM_BLUE_MAX_Y);
        } else {
            return (y >= PERIM_RED_MIN_Y && y <= PERIM_RED_MAX_Y);
        }
    }

    private void transitionTo(State newState) {
        log(getName(), String.format(Locale.US, "State: %s -> %s", currentState, newState));
        currentState = newState;
        stateTimer.reset();
    }

    private double getDistanceMoved() {
        Pose2D current = drivetrain.getPose();
        return Math.hypot(current.getX(DistanceUnit.INCH) - startPose.getX(DistanceUnit.INCH),
                current.getY(DistanceUnit.INCH) - startPose.getY(DistanceUnit.INCH));
    }

    @Override
    public boolean isFinished() {
        return currentState == State.DONE || globalTimer.seconds() > 12.0; // Increased timeout for initial rotation
    }

    @Override
    public void end() {
        if (intake != null) {
            intake.enableCameraSensing(false);
        }
        if (drivetrain != null) {
            drivetrain.stop();
        }
        log(getName(), "Finished.");
    }
}