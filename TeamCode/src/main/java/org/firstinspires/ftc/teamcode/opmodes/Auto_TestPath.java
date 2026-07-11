package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.ArrayList;

/**
 * A test autonomous OpMode that uses the command-based structure
 * to execute a simple path.
 */
@Autonomous(name = "Autonomous Test Path", group = "Tests")
@Disabled
public class Auto_TestPath extends OpMode {

    // Subsystems
    private RobotHardware robot;
    private Drivetrain drivetrain;
//    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;

    // --- MODIFIED: Use the new CommandScheduler ---
    private CommandScheduler scheduler;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // 1. Re-initialize hardware and subsystems
        robot = new RobotHardware();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        drivetrain.setDebug(true);
//        intake = new Intake(robot);
//        shooter = new Shooter(robot);
//        turret = new Turret(robot);

        // --- NEW: Instantiate the CommandScheduler ---
        scheduler = new CommandScheduler(drivetrain);

        // 2. IMPORTANT: Reset odometry at the start of autonomous
        drivetrain.resetOdometry();
        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // 3. Build the sequence of commands
        buildAutonomousSequence();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // The scheduler will automatically start the first command on its first update cycle.
        // No action needed here anymore.
    }

    @Override
    public void loop() {
        // --- Subsystem Updates ---
        // ALWAYS update all subsystems in the main loop.
        drivetrain.update();
//        intake.update();
//        shooter.update(intake, turret, drivetrain);
//        turret.update(drivetrain);

        // --- Scheduler Update ---
        // The scheduler runs all the command logic.
        scheduler.update();

        // --- Telemetry ---
        updateTelemetry();
    }

    @Override
    public void stop() {
        // Clean up resources.
        if (scheduler != null) {
            scheduler.reset(); // This will end all active commands.
        }
        if (drivetrain != null) {
            drivetrain.closeDebug();
        }
        super.stop();
    }

    /**
     * This is where you define the actual sequence of actions for your autonomous routine.
     */
    private void buildAutonomousSequence() {
        // --- Path 1: Drive forward 50 inches ---
        ArrayList<CurvePoint> driveForwardPath = new ArrayList<>();
        driveForwardPath.add(new CurvePoint(0, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        driveForwardPath.add(new CurvePoint(50, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        FollowPathCommand path1 = new FollowPathCommand(driveForwardPath, Math.toRadians(90), true).withName("Drive forward");

        // --- Path 2: Strafe left 50 inches ---
        ArrayList<CurvePoint> strafeLeftPath = new ArrayList<>();
        strafeLeftPath.add(new CurvePoint(50, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        strafeLeftPath.add(new CurvePoint(50, 50, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        FollowPathCommand path2 = new FollowPathCommand(strafeLeftPath, Math.toRadians(0), true).withName("Strafe left");

        // --- Path 3: Strafe right 50 inches ---
        ArrayList<CurvePoint> strafeRightPath = new ArrayList<>();
        strafeRightPath.add(new CurvePoint(50, 50, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        strafeRightPath.add(new CurvePoint(50, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        FollowPathCommand path3 = new FollowPathCommand(strafeRightPath, Math.toRadians(180), true).withName("Strafe right");

        // --- Path 4: Drive backward 50 inches ---
        ArrayList<CurvePoint> driveBackwardPath = new ArrayList<>();
        driveBackwardPath.add(new CurvePoint(50, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        driveBackwardPath.add(new CurvePoint(0, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6));
        FollowPathCommand path4 = new FollowPathCommand(driveBackwardPath, Math.toRadians(-90), true).withName("Drive backward");

        // --- Add commands to the scheduler ---
        scheduler.add(path1);
        scheduler.add(path2);
        scheduler.add(path3);
        scheduler.add(path4);

//        // Code to test with conditional transition
//        ArrayList<CurvePoint> pathToGoal = new ArrayList<>();
//        pathToGoal.add(new CurvePoint(0, 0, 0.5, 0.5, 10, Math.toRadians(60), 0.6));
//        pathToGoal.add(new CurvePoint(60, -35, 0.5, 0.5, 10, Math.toRadians(60), 0.6));
//
//        // --- Create the command and set the transition condition in one line ---
//        scheduler.add(
//                new FollowPathCommand(pathToGoal, Math.toRadians(90), true)
//                        .transitionWhenDistancetoEndIsLessThan(12.0) // Start next command when 12 inches away
//        );
//
//        // The next command (e.g., AimTurretCommand) will start when the condition is met,
//        // even while the FollowPathCommand is still running to cover the last 12 inches.
//        scheduler.add(new AimTurretCommand(Alliance.Color.RED));
//
//            // In your Auto OpMode's buildAutonomousSequence() method...
//
//            scheduler.add(
//                    new DecodeObeliskCommand()
//                            .transitionImmediately() // This is the key change!
//            );
//
//    // Because of the immediate transition, the scheduler will start this path
//    // command on the very next loop, while DecodeObeliskCommand continues
//    // its scan in the background.
//            scheduler.add(new FollowPathCommand(pathToGoal, Math.toRadians(90)));
//
//    // Subsequent commands will wait for the FollowPathCommand to finish (or meet its own transition condition).
//            scheduler.add(new AimTurretCommand(Alliance.Color.BLUE));
//
//            scheduler.add(new ShootByPatternCommand()); // This will now use the correct pattern, even if the scan timed out.

//        // In your autonomous OpMode's runOpMode() method or an equivalent setup method...
//
//// Create paths for different segments of the autonomous routine
//        ArrayList<CurvePoint> pathToFirstBall = ... ;
//        ArrayList<CurvePoint> pathToShootingSpot = ... ;
//        ArrayList<CurvePoint> pathToPark = ... ;
//
//// Add commands to the scheduler with descriptive names
//        scheduler.add(new DecodeObeliskCommand().withName("Start Obelisk Scan"));
//
//        scheduler.add(new FollowPathCommand(pathToFirstBall, Math.toRadians(90))
//                .withName("Drive to First Ball")
//                .transitionWhenDistancetoEndIsLessThan(10)); // Transition early to start intake
//
//        scheduler.add(new FollowPathCommand(pathToShootingSpot, Math.toRadians(45))
//                .withName("Drive to Shooting Spot"));
//
//        scheduler.add(new AimTurretCommand(Alliance.Color.RED).withName("Aim Turret"));
//
//        scheduler.add(new ShootByPatternCommand().withName("Shoot All Balls"));
//
//        scheduler.add(new FollowPathCommand(pathToPark, Math.toRadians(0))
//                .withName("Park"));


    }

    /**
     * Provides driver station telemetry.
     */
    private void updateTelemetry() {
        // We can check if the scheduler is busy to see if the sequence is running
        if (scheduler.isBusy()) {
            telemetry.addData("Auto Status", "Running sequence...");
        } else {
            telemetry.addData("Auto Status", "Sequence Finished");
        }

        Pose2D currentPose = drivetrain.getPose();
        telemetry.addData("X (in)", "%.2f", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.2f", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.2f", currentPose.getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }
}

