package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FieldSimulator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Blackboard;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp(name = "TeleOp Robot Main", group = "Main")
public class TeleOp_Main extends OpMode {

    // --- Subsystem and Hardware Declarations ---
    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Intake intake;
////    private Shooter shooter;
//    private Turret turret;
    private FieldSimulator fieldSimulator;
    private CommandScheduler scheduler;

    private Alliance.Color currentAlliance;

    // --- Constants for Driver Override ---
    private static final double STICK_THRESHOLD = 0.15; // Stick movement past this cancels auto-paths

    private enum TurretAimingState {
        MANUAL,
        AIM_RED,
        AIM_BLUE,
        AIM_ODOMETRY
    }
    private TurretAimingState turretState = TurretAimingState.MANUAL;

    private ElapsedTime turretOverrideTimer = new ElapsedTime();
    private boolean isTurretOverridden = false;
    private static final double TURRET_OVERRIDE_TIMEOUT_S = 1.0;
    private static final double TURRET_OVERRIDE_STICK_THRESHOLD = 0.1;

    public static final boolean DEBUG_ENABLED = true;
    private enum DebugMode {
        OFF,
        FLYWHEEL,
        TURRET_AIM,
        BALL_COLOR_LEFT,
        BALL_COLOR_MID,
        BALL_COLOR_RIGHT,
        FIELD_SIM
    }
    private DebugMode currentDebugMode = DebugMode.OFF;

    // --- Constants and State for End Game Rumble Alerts ---
    private static final int RUMBLE_DURATION_MS = 2000; // 2 second rumble
    // A standard FTC match is 120 seconds.
    private static final double END_GAME_START_TIME_S = 90.0; // 30 seconds before end
    private static final double FINAL_COUNTDOWN_TIME_S = 113.0; // 7 seconds before end
    private static final double LIFTER_ACTIVATION_TIME_S = 110.0; // Only allow lifter in the last 10 seconds
    private boolean hasRumbledForEndGame = false;
    private boolean hasRumbledForFinalCountdown = false;

    // --- Performance and Timing ---
    private ElapsedTime runtime = new ElapsedTime();
    private MovingStatistics loopTimes = new MovingStatistics(100);
    private ElapsedTime loopTimer = new ElapsedTime();

    // --- State variable to manage auto-pattern return ---
//    private Shooter.ShootingMode shooterModeBeforeAuto = Shooter.ShootingMode.PRELOAD;

    // Enum to handle the three possible automated destinations
    private enum PathDestination {
        PUSH_GATE,
        RETURN_SHOOT,
        OPEN_GATE
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // Check if we are starting fresh or coming from an Autonomous run
        boolean hasAutoData = blackboard.containsKey(Blackboard.ROBOT_POSE_KEY);

        // --- Retrieve the alliance color from the blackboard ---
        // It gets the value stored from Auto, or defaults to BLUE if not found.
        Object allianceFromAuto = blackboard.getOrDefault(Blackboard.ALLIANCE_KEY, Alliance.Color.BLUE);
        // We must cast the retrieved object back to its original type.
        this.currentAlliance = (Alliance.Color) allianceFromAuto;
        if ( this.currentAlliance == null) {
            System.out.println("TELEOP INIT: Null alliance. Defaulting to BLUE.");
            this.currentAlliance =  Alliance.Color.BLUE;
        }


        if (this.currentAlliance == Alliance.Color.RED) {
            this.turretState = TurretAimingState.AIM_RED;
        } else {
            this.turretState = TurretAimingState.AIM_BLUE;
        }

        // 1. Initialize the hardware hub
        robot = new RobotHardware();
        robot.init(hardwareMap);

        // 2. Initialize all subsystems by passing the hardware hub
        drivetrain = new Drivetrain(robot);
        intake = new Intake(robot);
//        shooter = new Shooter(robot);
//        turret = new Turret(robot);
        fieldSimulator = new FieldSimulator();
        scheduler = new CommandScheduler(drivetrain);

        // Tell the scheduler to stop acting like it's in Autonomous mode
        scheduler.setTeleOpMode(true);

        // If no Auto was run, explicitly reset odometry to ensure a clean 0,0,0 starting point.
        if (!hasAutoData) {
            drivetrain.resetOdometry();
            System.out.println("TELEOP INIT: No Auto data found. Odometry and IMU reset to 0.");
            gamepad1.rumble(2000); // Sharp 0.5 second vibrate to indicate error
        }

        // 1. Load the robot's final pose from Auto, or default to (0,0,0) if not found.
        Pose2D initialPose = (Pose2D) blackboard.getOrDefault(
                Blackboard.ROBOT_POSE_KEY,
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)
        );

        // 2. Load the turret's final angle from Auto, or default to 0.0 if not found.
//        double initialTurretAngle = (double) blackboard.getOrDefault(
//                Blackboard.TURRET_ANGLE_KEY,
//                0.0
//        );

        // 3. Set the loaded positions in the subsystems.
        drivetrain.setPosition(initialPose);
//        turret.setInitialPosition(initialTurretAngle);

        System.out.printf(Locale.US, "TELEOP INIT: Loaded state. Pose: (%.1f, %.1f, %.1f)%n",
                initialPose.getX(DistanceUnit.INCH),
                initialPose.getY(DistanceUnit.INCH),
                initialPose.getHeading(AngleUnit.DEGREES));

//        shooter.setShootingMode(Shooter.ShootingMode.PRELOAD);
//        shooter.setFlywheelMode(Shooter.FlywheelMode.AUT

        if (DEBUG_ENABLED) {
            setDebugMode(DebugMode.OFF);
        }

//        // Set the shooter to PRELOAD mode by default for TeleOp.
//        shooter.setShootingMode(Shooter.ShootingMode.PRELOAD);
//        // Set the flywheel to AUTO mode by default for TeleOp.
//        shooter.setFlywheelMode(Shooter.FlywheelMode.AUTO);
//
//        // 3. Set the robot's starting position
//        // This tells the robot it is starting at the field origin (0,0) with a 0-degree heading.
//        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//
//        if (DEBUG_ENABLED) {
//            setDebugMode(DebugMode.OFF);
//        }

        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData("Detected Alliance", currentAlliance.toString());
        telemetry.addLine("\n--- Gamepad 1 (Driver) ---");
        telemetry.addLine("Left Stick: Drive | Right Stick: Turn");
        telemetry.addLine("Start: Full System ON | Back: Full System OFF");
//        telemetry.addLine("Y: Cycle Feeder (Off/Fwd/Rev)");
        telemetry.addLine("Y: [AUTO] Open Gate | B: [AUTO] Push Gate | X: [AUTO] Return Shoot");
        telemetry.addLine("D-Pad U/D: Lifter Down/Up");
        // telemetry.addLine("D-Pad U/D: Adjust Roller Bed Speed");
        telemetry.addLine("D-Pad Right: Toggle Flywheel AUTO/MANUAL");
        telemetry.addLine("A: Reset Odo Pos (must be heading 0)");
//        telemetry.addLine("X/A/B: Toggle Left/Mid/Right Gates");
        telemetry.addLine("\n--- Gamepad 2 (Operator) ---");
        telemetry.addLine("Right Stick: Manual Turret (when not in auto-aim)");
        telemetry.addLine("Left Stick U/D: Shooter Angle");
//        telemetry.addLine("D-Pad U/D: Flywheel RPM (+/-)");
//        telemetry.addLine("D-Pad Up: Flywheel RPM (+)");
        telemetry.addLine("D-Pad Down: HOLD for Odometry Aim");
        // telemetry.addLine("D-Pad U/D: Open/Close ALL GATES");
//        telemetry.addLine("D-Pad L/R: Booster Speed (+/-)");
        telemetry.addLine("D-Pad L/R: Flywheel RPM (-/+)");
        telemetry.addLine("Right Bumper: Toggle Shooting Mode (Preload/Manual)");
        telemetry.addLine("Right Trigger: PULL TRIGGER to shoot pre-loaded ball");
        telemetry.addLine("Left Bumper: Start AUTO PATTERN sequence");
        telemetry.addLine("Left Trigger: Discard Ball");
//        telemetry.addLine("X, A, B: Open Left, Mid, Right Gates");
        telemetry.addLine("X: Close Shot | A: Mid Shot | B: Far Shot");

        telemetry.addLine("Options: Cycle Turret Mode (Manual -> Aim Red -> Aim Blue)");
        telemetry.addLine("Y: Toggle Intake Roller");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // It's good practice to call reset for subsystems that need it.
//        shooter.reset();
    }

    @Override
    public void start() {
        runtime.reset();
        loopTimer.reset();

        // --- Reset rumble flags at the start of the match ---
        hasRumbledForEndGame = false;
        hasRumbledForFinalCountdown = false;

//        shooter.setFlywheelRPM(500);
//        intake.toggleIntakeRoller();
////        intake.setRollerBedSpeed(1);
//        intake.setFeederState(Intake.FeederState.REVERSE);
        intake.changeState(Intake.IntakeState.FORWARD);

        // If the initial state is an aiming state, tell the turret what to do.
//        if (turretState == TurretAimingState.AIM_RED) {
//            turret.aimAtTag(Alliance.Color.RED);
//        } else if (turretState == TurretAimingState.AIM_BLUE) {
//            turret.aimAtTag(Alliance.Color.BLUE);
//        }
//
//        shooter.setFlywheelRPM(0);
//        shooter.setBoosterSpeed(0.0);
//        intake.setIntakeRoller(true);
//        intake.setRollerBedSpeed(0.0);
//        intake.setFeederState(Intake.FeederState.OFF);


//        intake.toggleGate(0);
//        intake.toggleGate(1);
//        intake.toggleGate(2);

    }

    @Override
    public void loop() {
        loopTimer.reset();

        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        // Update the command scheduler to process automated paths
        scheduler.update();

        // --- Monitor the auto-pattern sequence ---
        monitorAutoPattern();

        // --- Handle the end-game rumble alerts ---
        handleRumbleAlerts();

        // --- Handle Gamepad Inputs ---
        handleDriverControls();   // Gamepad 1
        handleOperatorControls(); // Gamepad 2

        // --- Subsystem Updates ---
        // Call the update() method for each subsystem on every loop.
        // This is where continuous logic, like PID loops and timers, is executed.
        drivetrain.update();
        intake.update();
//        shooter.update(intake, turret, drivetrain);
//        turret.update(drivetrain);
        fieldSimulator.update(drivetrain);

        // --- Telemetry Update ---
        updateTelemetry();

        // Record loop time for performance monitoring
        loopTimes.add(loopTimer.nanoseconds());
    }

    /**
     * Handles all inputs from Gamepad 1 (Driver).
     */

    private void handleDriverControls() {
        // Capture stick inputs
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        // --- MANUAL OVERRIDE ---
        // If the driver moves the sticks while an auto-path is running, cancel it.
        if (drivetrain.isBusy()) {
            if (Math.abs(driveY) > STICK_THRESHOLD ||
                    Math.abs(driveX) > STICK_THRESHOLD ||
                    Math.abs(turn) > STICK_THRESHOLD) {

                drivetrain.stop(); // Stops the movement logic
                scheduler.cancelAll(); // Clears the command from the scheduler
                telemetry.addLine("OVERRIDE: Path Cancelled");
            }
        }

        // --- DRIVETRAIN CONTROL ---
        // Only drive manually if NOT busy with an automated path
        if (!drivetrain.isBusy()) {
            drivetrain.drive(driveY, driveX, turn);
        }

        // --- AUTOMATED PATH TRIGGERS ---

        // Button Y: Open the Gate (New Functionality)
        if (gamepad1.yWasPressed()) {
            // NOTE: Current functionality 'intake.cycleFeederState()' has been replaced by Open Gate path
            // if (gamepad1.yWasPressed()) { intake.cycleFeederState(); }
//            executePredefinedPath(PathDestination.OPEN_GATE);
            intake.changeState(Intake.IntakeState.OFF);
        }

        // Button B: Drive to and Push Ramp Gate
        if (gamepad1.bWasPressed()) {
//            executePredefinedPath(PathDestination.PUSH_GATE);
            intake.changeState(Intake.IntakeState.FORWARD);
        }

        // Button X: Return to Shooting Position
        if (gamepad1.xWasPressed()) {
//            executePredefinedPath(PathDestination.RETURN_SHOOT);
            intake.changeState(Intake.IntakeState.REVERSE);
        }

//        // Drivetrain
//        drivetrain.drive(
//                -gamepad1.left_stick_y,
//                gamepad1.left_stick_x * 1.1, // Original scaling factor
//                -gamepad1.right_stick_x
//        );

//        // Drivetrain and Intake adjustments
//        if (gamepad1.yWasPressed()) { intake.cycleFeederState(); }

        // Only allow the lifter to be used in the final seconds of the match.
        if (runtime.seconds() >= LIFTER_ACTIVATION_TIME_S) {
            if (gamepad1.dpadUpWasPressed()) {
                // Execute the full end-game sequence when deploying the lifter.

                // 1. Set the turret to a safe position (-75 degrees)
//                turret.setPosition(-75.0);
//
//                // 2. Set flywheel to MANUAL and stop it.
//                shooter.setFlywheelMode(Shooter.FlywheelMode.MANUAL);
//                shooter.setFlywheelRPM(0);
//
//                // 3. Set shooting mode to MANUAL and retract the shooter hood to its minimum angle.
//                shooter.setShootingMode(Shooter.ShootingMode.MANUAL);
//                shooter.setAngle(0.11); // Directly using the known minimum angle
//
//                // 4. Stop the intake
//                intake.stop();

                // Finally, deploy the lifter.
//                drivetrain.setLifterDown(); // Move lifter to the 'down' position
            }
//            if (gamepad1.dpadDownWasPressed()) {
//                drivetrain.setLifterUp(); // Move lifter to the 'up' position
//            }
        }
        // LEAVE THIS CODE IN CASE WE WANT TO USE UP/DOWN TO ADJUST THE ROLLER BED SPEED
        //        if (gamepad1.dpadUpWasPressed()) { intake.adjustRollerBedSpeed(true); }
        //        if (gamepad1.dpadDownWasPressed()) { intake.adjustRollerBedSpeed(false); }

        // --- Toggle Flywheel AUTO/MANUAL Mode ---
//        if (gamepad1.dpadRightWasPressed()) {
//            Shooter.FlywheelMode currentFlywheelMode = shooter.getFlywheelMode();
//            if (currentFlywheelMode == Shooter.FlywheelMode.MANUAL) {
//                shooter.setFlywheelMode(Shooter.FlywheelMode.AUTO);
//            } else {
//                shooter.setFlywheelMode(Shooter.FlywheelMode.MANUAL);
//            }
//        }

        // When START is pressed, turn on all intake and shooting systems to full power.
//        if (gamepad1.startWasPressed()) {
//            shooter.setFlywheelMode(Shooter.FlywheelMode.MANUAL);
//            shooter.setFlywheelRPM(3000);
//            shooter.setBoosterSpeed(-1.0);
//            intake.setIntakeRoller(true);
//            intake.setRollerBedSpeed(1.0);
//            intake.setFeederState(Intake.FeederState.FORWARD);
//        }
//
//        // When BACK is pressed, turn off all intake and shooting systems.
//        if (gamepad1.backWasPressed()) {
//            shooter.setFlywheelMode(Shooter.FlywheelMode.MANUAL);
//            shooter.setFlywheelRPM(0);
//            shooter.setBoosterSpeed(0.0);
//            intake.setIntakeRoller(false);
//            intake.setRollerBedSpeed(0.0);
//            intake.setFeederState(Intake.FeederState.OFF);
//        }

        if (gamepad1.aWasPressed()) {
            // Attempt to reset position and capture success/failure
//            boolean success = drivetrain.resetPositionWithVision(turret);

            // If vision was not found or calculation failed, alert the driver
//            if (!success) {
//                gamepad1.rumble(500); // Sharp 0.5 second vibrate to indicate error
//            }
        }
    }

    /**
     * Builds and schedules an alliance-specific path.
     * @param destination The target location for the robot.
     */
    private void executePredefinedPath(PathDestination destination) {
        ArrayList<CurvePoint> path = new ArrayList<>();
        Pose2D currentPose = drivetrain.getPose();

        // Step 1: Start with current position (Required for smooth transition)
        path.add(new CurvePoint(
                currentPose.getX(DistanceUnit.INCH),
                currentPose.getY(DistanceUnit.INCH),
                1, 1, 5.0, Math.toRadians(60), 0.9
        ));

        double followAngle = 90.0; // Default
        String name = "";

        // Prepare the system
        drivetrain.stop();
        scheduler.cancelAll();

        // Step 2: Define coordinates based on alliance and destination
        if (currentAlliance == Alliance.Color.BLUE) {
            followAngle = 90.0;
            switch (destination) {
                case OPEN_GATE:
                    name = "Blue: Open Gate";
                    followAngle = 0;
                    path.add(new CurvePoint(0.00, 45.54, 1, 1, 5.00, Math.toRadians(60.0), 0.90));
                    path.add(new CurvePoint(0.00, 60.36, 1, 1, 5.00, Math.toRadians(60.0), 0.90));
                    break;
                case PUSH_GATE:
                    name = "Blue: Push Gate";
                    path.add(new CurvePoint(-16.56, 40.86, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-18.18, 48.24, 0.80, 0.80, 5.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-17.10, 54.54, 0.60, 0.60, 5.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-10.44, 59.94, 0.40, 0.40, 5.00, Math.toRadians(60.0), 0.60));
                    break;
                case RETURN_SHOOT:
                    name = "Blue: Return Shoot";
                    path.add(new CurvePoint(-10.44, 59.94, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-14.04, 57.06, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-10.44, 44.10, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(26.10, 28.44, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(24.84, 17.64, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    break;
            }
        } else {
            // RED ALLIANCE (Mirrored Y)
            followAngle = 90.0;
            switch (destination) {
                case OPEN_GATE:
                    name = "Red: Open Gate";
                    followAngle = 180;
                    path.add(new CurvePoint(0.00, -45.54, 1, 1, 5.00, Math.toRadians(60.0), 0.90));
                    path.add(new CurvePoint(0.00, -60.36, 1, 1, 5.00, Math.toRadians(60.0), 0.90));
                    break;
                case PUSH_GATE:
                    name = "Red: Push Gate";
                    path.add(new CurvePoint(-16.56, -40.86, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-18.18, -48.24, 0.80, 0.80, 5.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-17.10, -54.54, 0.60, 0.60, 5.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-10.44, -59.94, 0.40, 0.40, 5.00, Math.toRadians(60.0), 0.60));
                    break;
                case RETURN_SHOOT:
                    name = "Red: Return Shoot";
                    path.add(new CurvePoint(-10.44, -59.94, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-14.04, -57.06, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(-10.44, -44.10, 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(26.10, -28.44, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    path.add(new CurvePoint(24.84, -17.64, 1, 1, 10.00, Math.toRadians(60.0), 0.60));
                    break;
            }
        }

        // Schedule the command
        scheduler.add(new FollowPathCommand(path, Math.toRadians(followAngle), true)
                .withName(name));
    }

    /**
     * Handles all inputs from Gamepad 2 (Operator).
     */
    private void handleOperatorControls() {
        // --- Auto-Pattern Trigger ---
        if (gamepad2.leftBumperWasPressed()) {
//            intake.closeAllGates();
//            intake.setShakeMode(true);
        }
        if (gamepad2.leftBumperWasReleased()) {
            // Only start if we are not already in the auto pattern mode
//            if (shooter.getShootingMode() != Shooter.ShootingMode.AUTO_PATTERN) {
////                // Set the bed back to full speed
////                intake.setShakeMode(false);
////                intake.setRollerBedSpeed(1.0);
//
//                // Store the current mode so we can return to it
//                shooterModeBeforeAuto = shooter.getShootingMode();
//                // Start the sequence
//                Turret.DecodePattern pattern = Turret.getDecodedGamePattern();
//                shooter.startAutoPattern(intake, pattern);
//            }
        }

        // Turret and Shooter
        // The Options button cycles through the aiming modes.
        if (gamepad2.optionsWasPressed()) {
            if (turretState == TurretAimingState.MANUAL) {
                turretState = TurretAimingState.AIM_RED;
                // --- Call aimAtTag ONCE when we enter the state ---
//                turret.aimAtTag(Alliance.Color.RED);
                currentAlliance = Alliance.Color.RED;
            } else if (turretState == TurretAimingState.AIM_RED) {
                turretState = TurretAimingState.AIM_BLUE;
                // --- Call aimAtTag ONCE when we enter the state ---
//                turret.aimAtTag(Alliance.Color.BLUE);
                currentAlliance = Alliance.Color.BLUE;
            } else { // It's AIM_BLUE
                turretState = TurretAimingState.MANUAL;
                // When returning to manual, we can just let the stick input take over.
            }
        }

        // --- Turret override logic ---
        boolean isStickMoved = Math.abs(gamepad2.right_stick_x) > TURRET_OVERRIDE_STICK_THRESHOLD;
        // Condition to ENTER override mode
        if (turretState != TurretAimingState.MANUAL && isStickMoved) {
            isTurretOverridden = true;
        }
        if (isTurretOverridden) {
            // While overridden, always use manual power.
//            turret.setManualPower(-gamepad2.right_stick_x);

            // If stick is moved, reset the timeout timer.
            if (isStickMoved) {
                turretOverrideTimer.reset();
            }

            // If stick has been centered for long enough, exit override mode.
            if (turretOverrideTimer.seconds() > TURRET_OVERRIDE_TIMEOUT_S) {
                isTurretOverridden = false;
                // Re-engage the auto-aim for the current state.
                if (turretState == TurretAimingState.AIM_RED) {
//                    turret.aimAtTag(Alliance.Color.RED);
                } else if (turretState == TurretAimingState.AIM_BLUE) {
//                    turret.aimAtTag(Alliance.Color.BLUE);
                }
            }
        } else {
            // --- This is the NORMAL behavior when not overridden ---
            if (turretState == TurretAimingState.MANUAL) {
//                turret.setManualPower(-gamepad2.right_stick_x);
            }
            // In AIM_RED or AIM_BLUE mode, do nothing. The turret's own update loop is handling it.
        }

//        shooter.adjustAngle(gamepad2.left_stick_y);

          // Open and close all intake gates
         // if (gamepad2.dpadUpWasPressed()) { intake.openAllGates(); }
         // if (gamepad2.dpadDownWasPressed()) { intake.closeAllGates(); }
         // Old functionality: Commented out to prevent conflicts
//         if (gamepad2.dpadUpWasPressed()) { shooter.adjustFlywheelRPM(true); }
         // if (gamepad2.dpadDownWasPressed()) { shooter.adjustFlywheelRPM(false); }

        // D-Pad Down is now a "hold" button for odometry aiming.
        if (gamepad2.dpadDownWasPressed()) {
            // Can only enter odo-aim if currently in a vision-aim state.
            if (turretState == TurretAimingState.AIM_BLUE || turretState == TurretAimingState.AIM_RED) {
                System.out.println("OPERATOR: Engaging ODOMETRY_AIM fallback.");
                turretState = TurretAimingState.AIM_ODOMETRY;
                // Use the globally stored alliance color to aim.
//                turret.aimWithOdometry(this.currentAlliance);
            }
        }
        if (gamepad2.dpadDownWasReleased()) {
            // When released, always revert to the primary vision-based aiming.
            if (turretState == TurretAimingState.AIM_ODOMETRY) {
                System.out.println("OPERATOR: Reverting to vision-based AUTO_AIM.");
                // Re-engage the correct vision-aim state.
                if (this.currentAlliance == Alliance.Color.RED) {
                    turretState = TurretAimingState.AIM_RED;
//                    turret.aimAtTag(Alliance.Color.RED);
                } else {
                    turretState = TurretAimingState.AIM_BLUE;
//                    turret.aimAtTag(Alliance.Color.BLUE);
                }
            }
        }

        // D-Pad L/R now adjusts flywheel RPM
//        if (gamepad2.dpadRightWasPressed()) { shooter.adjustFlywheelRPM(true); } // Increase RPM
//        if (gamepad2.dpadLeftWasPressed()) { shooter.adjustFlywheelRPM(false); } // Decrease RPM
//        // --- Old functionality is commented out ---
//        //if (gamepad2.dpadRightWasPressed()) { shooter.adjustBoosterSpeed(true); }
//        //if (gamepad2.dpadLeftWasPressed()) { shooter.adjustBoosterSpeed(false); }
//
//        // Intake and Gating
//       if (gamepad2.yWasPressed()) { intake.toggleIntakeRoller(); }
//        if (gamepad2.x) { intake.openGate(0); } // Open Left
//        if (gamepad2.a) { intake.openGate(1); } // Open Mid
//        if (gamepad2.b) { intake.openGate(2); } // Open Right

        // Pass-through mode (hold left bumper)
        // intake.setPassThrough(gamepad2.left_bumper);

//        if (gamepad2.xWasPressed()) { // Close Shot
//            shooter.setShootingSpot(Shooter.ShootingSpot.CLOSE);
//        }
//        if (gamepad2.aWasPressed()) { // Mid-Field Shot
//            shooter.setShootingSpot(Shooter.ShootingSpot.MID);
//        }
//        if (gamepad2.bWasPressed()) { // Far Shot
//            shooter.setShootingSpot(Shooter.ShootingSpot.FAR);
//        }

        // Right Bumper toggles the shooting mode
        if (gamepad2.rightBumperWasPressed()) {
//            if (shooter.getShootingMode() == Shooter.ShootingMode.PRELOAD) {
//                shooter.setShootingMode(Shooter.ShootingMode.MANUAL);
//            } else {
//                shooter.setShootingMode(Shooter.ShootingMode.PRELOAD);
//            }
        }

        // Right Trigger pulls the trigger to fire a pre-loaded ball
        if (gamepad2.right_trigger > 0.5) {
//            shooter.pullTrigger();
        }

        // Left Trigger pulls the discard trigger.
        if (gamepad2.left_trigger > 0.5) {
//            shooter.pullTriggerDiscardBall();
        }

//        // Decode scan mode to get the obelisk color pattern
//        if (gamepad2.backWasPressed()) {
//            turret.startDecodeScan();
//        }

        if (DEBUG_ENABLED && gamepad2.backWasPressed()) {
            int nextModeIndex = (currentDebugMode.ordinal() + 1) % DebugMode.values().length;
            setDebugMode(DebugMode.values()[nextModeIndex]);
        }
    }

    /**
     * Monitors the auto-pattern sequence and reverts to the previous
     * shooting mode when it's finished.
     */
    private void monitorAutoPattern() {
        // If we are currently in the auto-pattern mode...
//        if (shooter.getShootingMode() == Shooter.ShootingMode.AUTO_PATTERN) {
//            // ...and the shooter reports that the sequence is finished...
//            if (shooter.isAutoPatternFinished()) {
//                // ...then revert the shooter to its previous mode.
////                telemetry.addLine("Auto-Pattern Finished. Reverting to previous mode.");
//                shooter.setShootingMode(shooterModeBeforeAuto);
//            }
//        }
    }

    /**
     * Checks the match time and triggers rumble alerts for the drivers.
     */
    private void handleRumbleAlerts() {
        double currentTime = runtime.seconds();

        // Alert 1: End Game is starting (30 seconds left)
        if (!hasRumbledForEndGame && currentTime >= END_GAME_START_TIME_S) {
            //gamepad1.rumble(RUMBLE_DURATION_MS);
            gamepad2.rumble(RUMBLE_DURATION_MS);
            hasRumbledForEndGame = true; // Ensure this only runs once
        }

        // Alert 2: Final countdown (10 seconds left)
        if (!hasRumbledForFinalCountdown && currentTime >= FINAL_COUNTDOWN_TIME_S) {
//            gamepad1.rumble(RUMBLE_DURATION_MS);
            gamepad2.rumble(RUMBLE_DURATION_MS);
            hasRumbledForFinalCountdown = true; // Ensure this only runs once
        }
    }

    /**
     * Updates all telemetry data for the Driver Station.
     */
    private void updateTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Loop Freq (Hz)", "%.1f", 1 / (loopTimes.getMean() / 1e9));
        double timeRemaining = 120.0 - runtime.seconds();
        telemetry.addData("Time Remaining", "%.1f s", timeRemaining > 0 ? timeRemaining : 0);

//        telemetry.addData("Dist (LL)", "%.1f in", turret.getTargetDistanceByAngle());
//        telemetry.addData("Dist (Odo)", "%.1f in", turret.getTargetDistanceByOdometry(drivetrain));

        // Drivetrain Telemetry
        Pose2D pose = drivetrain.getPose();
        String posStr = String.format(Locale.US, "X:%.1f Y:%.1f H:%.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", posStr);

        // Intake Telemetry
//        intake.addIntakeCameraTelemetry(telemetry);
//        telemetry.addData("Intake On", intake.isIntakeOn());
//        telemetry.addData("Ball Colors", intake.getBallStatusString());
//        telemetry.addData("Feeder State", intake.getFeederState().toString());
//        telemetry.addData("Bed Speed", "%.2f", intake.getRollerBedSpeed());

        // Turret Telemetry with new aiming data
        telemetry.addData("Turret Mode", turretState.toString());
        if (turretState != TurretAimingState.MANUAL) {
//            telemetry.addData("  > Target Visible", turret.isTargetVisible());
//            telemetry.addData("  > Target Error (tx)", "%.2f degrees", turret.getTargetError());
//            telemetry.addData("  > Target Area (ta)", "%.2f %%", turret.getTargetArea());
//            telemetry.addData("  > On Target?", turret.isOnTarget());
        }
//        telemetry.addData("Turret HW", "Enc:%d (%.1f\u00B0), L-Limit:%b, R-Limit:%b",
//                turret.getCurrentPosition(),
//                turret.getCurrentAngle(),
//                turret.isLeftLimitPressed(),
//                turret.isRightLimitPressed());

        // Get the status object from the turret subsystem
//        LLStatus llStatus = turret.getLimelightStatus();
//        telemetry.addData("Limelight", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                llStatus.getTemp(), llStatus.getCpu(), (int)llStatus.getFps());
//
//        // Shooter Telemetry
//        telemetry.addData("Flywheel Mode", shooter.getFlywheelMode());
//        telemetry.addData("Flywheel", "Tgt: %.0f RPM | Curr: %.0f RPM",
//                shooter.getTargetRPM(), shooter.getCurrentRPM());
//        telemetry.addData("Shooter Angle", "%.2f", shooter.getShooterAngle());
//        telemetry.addData("Booster Speed", "%.2f", shooter.getBoosterSpeed());
//
//        telemetry.addData("Shooter Mode", shooter.getShootingMode());
//        telemetry.addData("Ball Chambered?", shooter.isBallChambered());
//
//        telemetry.addData("Game Pattern", Turret.getDecodedGamePattern().toString());

        if (DEBUG_ENABLED) {
            telemetry.addData("Debug Mode", currentDebugMode.toString());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Call the stop() method for each subsystem to ensure all motors are turned off.
        drivetrain.stop();
//        intake.stop();
//        shooter.stop();
//        turret.stop();
    }

    /**
     * Manages the activation and deactivation of debug modes across all subsystems.
     * @param mode The DebugMode to switch to.
     */
    private void setDebugMode(DebugMode mode) {
        if (!DEBUG_ENABLED) return;

        currentDebugMode = mode;

        // 1. Turn ALL subsystem debug features OFF first.
//        shooter.setDebug(false);
//        intake.setDebug(-1); // Use -1 to disable all color sensors
//        turret.setDebug(false); // Turn turret debug off by default
        if (fieldSimulator != null) fieldSimulator.setActive(false);

        // 2. Turn ON the specific mode that was requested.
        switch (mode) {
            case OFF:
                // All are already off, do nothing.
                break;
            case FLYWHEEL:
//                shooter.setDebug(true);
                break;
            case TURRET_AIM:
//                turret.setDebug(true);
                break;
            case BALL_COLOR_LEFT:
//                intake.setDebug(0);
                break;
            case BALL_COLOR_MID:
//                intake.setDebug(1);
                break;
            case BALL_COLOR_RIGHT:
//                intake.setDebug(2);
                break;
            case FIELD_SIM:
                if (fieldSimulator != null) fieldSimulator.setActive(true);
                break;
        }
    }
}
