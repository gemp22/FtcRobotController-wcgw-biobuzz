package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Corrected imports pointing to the new 'util' package
import org.firstinspires.ftc.teamcode.util.ListenForCurvePoints;
import org.firstinspires.ftc.teamcode.util.ListenResult;
import org.firstinspires.ftc.teamcode.util.ListenStatus;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Queue;

@TeleOp(name = "Live Path Tuner", group = "Tests")
//@Disabled
public class TeleOp_LivePathTuner extends OpMode {

    // --- Subsystems ---
    private RobotHardware robot;
    private Drivetrain drivetrain;
//    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;

    // --- Path Listening ---
    private ListenForCurvePoints listener;
    private boolean listeningOperationDone = false;
    private long listenerStartTime;
    private ArrayList<CurvePoint> receivedPath;
    private double receivedFollowAngle = 90.0;
    private Pose2D initialPose = null;
    private boolean isGoingHome = false; // NEW: Flag for "go home" mode

    // --- Command-based State Machine ---
    private final Queue<Command> commandQueue = new LinkedList<>();
    private Command currentCommand = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Subsystems...");
        telemetry.update();

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        drivetrain.setDebug(true);
//        intake = new Intake(robot);
//        shooter = new Shooter(robot);
//        turret = new Turret(robot);

        drivetrain.resetOdometry();
        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES,0));

        listener = ListenForCurvePoints.getInstance();
        listener.startListening();
        listenerStartTime = System.currentTimeMillis();

        telemetry.addData("Status", "Ready to Receive Path");
        telemetry.addLine("Send path from computer now...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!listeningOperationDone) {
            ListenResult result = listener.checkStatusAndGetPoints();

            telemetry.addData("Listener Status", result.getStatus());

            if (result.hasStartPose()) {
                telemetry.addData("Start Pose", String.format(Locale.US, "X:%.1f, Y:%.1f, H:%.1f",
                        result.getStartPoseX(), result.getStartPoseY(), result.getStartPoseHeading()));
            } else {
                telemetry.addData("Start Pose", "Not yet received");
            }

            // Display the received follow angle
            telemetry.addData("Follow Angle", "%.1f deg", result.getFollowAngle());

            telemetry.addData("Points Received", result.getPoints().size());
            double timeRemaining = (listenerStartTime + ListenForCurvePoints.TIMEOUT_MILLISECONDS - System.currentTimeMillis()) / 1000.0;
            telemetry.addData("Time Remaining (s)", "%.1f", timeRemaining);

            if (result.getStatus() != ListenStatus.LISTENING) {
                receivedPath = result.getPoints();
                receivedFollowAngle = result.getFollowAngle();
                listener.stopListening();
                listeningOperationDone = true;

                if (result.hasStartPose()) {
                    initialPose = new Pose2D(DistanceUnit.INCH, result.getStartPoseX(), result.getStartPoseY(), AngleUnit.DEGREES, result.getStartPoseHeading());

                    drivetrain.setPosition(initialPose);
                    telemetry.addLine("SUCCESS: Robot start pose has been set.");
                } else {
                    initialPose = new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.DEGREES, 0);

                    telemetry.addLine("WARNING: No start pose received. Using default (0,0,0).");
                }

                if (receivedPath != null && !receivedPath.isEmpty()) {
                    telemetry.addData("Status", "Path Received! " + receivedPath.size() + " points.");
                    telemetry.addLine("Ready to Start.");
                    // Pass the received angle to build the sequence
                    buildAutonomousSequence(receivedPath, receivedFollowAngle);
                } else {
                    telemetry.addData("Status", "No valid path received.");
                    telemetry.addData("Listener final state", result.getStatus());
                    if (result.getErrorMessage() != null) {
                        telemetry.addLine(result.getErrorMessage());
                    }
                }
            }
            telemetry.update();
        }
    }

    @Override
    public void start() {
//        intake.setIntakeRoller(true);
//        shooter.setShootingMode(Shooter.ShootingMode.PRELOAD);
        startNextCommand();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            goHome();
        }
        if (currentCommand != null) {
            if (currentCommand.isFinished()) {
                currentCommand.end();
                startNextCommand();
            } else {
                currentCommand.update();
            }
        }

        drivetrain.update();
//        intake.update();
//        shooter.update(intake, turret, drivetrain);
//        turret.update(drivetrain);
        updateTelemetry();
    }


    /**
     * Clears all commands and creates a new path from the current position back to the initial start pose.
     */
    private void goHome() {
        if (initialPose == null) {
            telemetry.addData("Status", "Cannot go home, initial position unknown.");
            telemetry.update();
            return;
        }

        telemetry.clear();
        telemetry.addData("Status", "Going home to initial start position!");
        isGoingHome = true; // NEW: Set the flag

        drivetrain.stop();
        commandQueue.clear();
        currentCommand = null;

        Pose2D currentPose = drivetrain.getPose();
        double startX = currentPose.getX(DistanceUnit.INCH);
        double startY = currentPose.getY(DistanceUnit.INCH);

        // The destination is the stored initial pose
        double endX = initialPose.getX(DistanceUnit.INCH);
        double endY = initialPose.getY(DistanceUnit.INCH);

        // Calculate the angle of the path segment from our current location to the home position.
        // This gives us the direction of travel in field coordinates.
        double pathAngleRadians = Math.atan2(endY - startY, endX - startX);
        double pathAngleDegrees = Math.toDegrees(pathAngleRadians);

        // The 'followAngle' is the desired robot heading relative to the path's direction.
        // We want the final robot heading to be the same as the initialPose's heading.
        // The path follower uses: target_heading = path_angle + (follow_angle - 90_deg)
        // We want: target_heading = initial_pose_heading
        // So: initial_pose_heading = path_angle + follow_angle - 90_deg
        // Solving for follow_angle: follow_angle = initial_pose_heading - path_angle + 90_deg
        double followAngle = initialPose.getHeading(AngleUnit.DEGREES) - pathAngleDegrees + 90.0;

        // NEW: Add telemetry for debugging
        telemetry.addData("Initial Pose Heading", "%.2f", initialPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Return Path Angle", "%.2f", pathAngleDegrees);
        telemetry.addData("Calculated Follow Angle", "%.2f", followAngle);
        telemetry.update();


        ArrayList<CurvePoint> homePath = new ArrayList<>();
        // Start point uses current robot coordinates with default parameters
        homePath.add(new CurvePoint(startX, startY, 0.5, 0.5, 10, Math.toRadians(45), 0.6));
        // End point is the initial start position of the tuned path
        homePath.add(new CurvePoint(endX, endY, 0.5, 0.5, 10, Math.toRadians(45), 0.6));

        // Add this new path to the queue and start it
        commandQueue.add(new FollowPathCommand(homePath, Math.toRadians(followAngle), true));
        startNextCommand();
    }

    @Override
    public void stop() {
        if (listener != null) {
            listener.stopListening();
        }
        if (drivetrain != null) {
            drivetrain.stop();
            drivetrain.closeDebug();
        }
        super.stop();
    }

    private void buildAutonomousSequence(ArrayList<CurvePoint> path, double followAngleDegrees) {
        commandQueue.clear();
        isGoingHome = false; // Reset flag when building a new sequence
        // Use the angle received from the simulator
        commandQueue.add(new FollowPathCommand(path, Math.toRadians(followAngleDegrees), true));
    }

    private void startNextCommand() {
        isGoingHome = false; // Reset the flag whenever a command starts
        currentCommand = commandQueue.poll();
        if (currentCommand != null) {
            currentCommand.start(drivetrain);
        }
    }

    private void updateTelemetry() {
        if (listeningOperationDone) {
            if (currentCommand != null) {
                telemetry.addData("Current Command", currentCommand.getClass().getSimpleName());
            } else {
                telemetry.addData("Current Command", "None (Sequence Finished)");
                isGoingHome = false; // Also reset flag when the queue is empty
            }
            telemetry.addData("Commands in Queue", commandQueue.size());

            Pose2D currentPose = drivetrain.getPose();
            telemetry.addData("X (in)", "%.2f", currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES)));

            // Add the goHome flag status to telemetry
            telemetry.addData("Mode", isGoingHome ? "Going Home" : "Running Path");
        }
        telemetry.update();
    }
}
