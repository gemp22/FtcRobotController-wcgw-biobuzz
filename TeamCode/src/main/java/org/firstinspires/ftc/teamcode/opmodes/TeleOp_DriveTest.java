package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Locale;

/**
 * This OpMode is designed to test the Drivetrain subsystem, specifically the
 * new driveForward() and strafe() methods.
 *
 * It allows for standard joystick control but overrides it with fixed-distance
 * movements when specific buttons are pressed.
 */
@TeleOp(name = "TeleOp Drivetrain Test", group = "Test")
@Disabled
public class TeleOp_DriveTest extends OpMode {

    private RobotHardware robot;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Drivetrain Test...");

        // 1. Initialize hardware and the drivetrain subsystem
        robot = new RobotHardware();
        robot.init(hardwareMap);
        drivetrain = new Drivetrain(robot);

        // 2. Set a starting position (optional but good practice)
        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // 3. Provide instructions on the Driver Station
        telemetry.addLine("--- Drivetrain Test Controls ---");
        telemetry.addLine("Left Stick: Drive | Right Stick: Turn");
        telemetry.addLine("D-Pad Up/Down: Drive Fwd/Rev 2/4/6 inches");
        telemetry.addLine("D-Pad Left/Right: Strafe Left/Right 2/4/6 inches");
        telemetry.addLine("X: 2 inches | A: 4 inches | B: 6 inches");
        telemetry.addLine("Start: Reset Odometry");
        telemetry.addData("Status", "Ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset odometry on start to ensure a clean slate
        drivetrain.resetOdometry();
    }

    @Override
    public void loop() {
        // --- Subsystem Updates ---
        // Always call the drivetrain's update method. This handles odometry
        // and executes any active autonomous movements.
        drivetrain.update();

        // --- Handle Gamepad Inputs ---
        handleTestControls();

        // --- Telemetry Update ---
        updateTelemetry();
    }

    private void handleTestControls() {
        // --- Autonomous Movement Control ---
        // Only allow new autonomous commands if the drivetrain is NOT already busy.
        if (!drivetrain.isBusy()) {
            // Check for button presses to trigger fixed movements.
            // The value (2, 4, or 6) is determined by which face button is held.
            double distance = 0;
            if (gamepad1.x) distance = 2.0;
            if (gamepad1.a) distance = 4.0;
            if (gamepad1.b) distance = 6.0;

            if (distance > 0) {
                if (gamepad1.dpad_up) {
                    drivetrain.driveForward(distance);
                } else if (gamepad1.dpad_down) {
                    drivetrain.driveForward(-distance);
                } else if (gamepad1.dpad_left) {
                    drivetrain.strafe(distance); // Positive inches is left
                } else if (gamepad1.dpad_right) {
                    drivetrain.strafe(-distance); // Negative inches is right
                }
            }
        }

        // --- Manual Drive Control ---
        // Only allow manual joystick driving if the drivetrain is NOT busy with an autonomous move.
        // This prevents the joystick from interfering with the driveForward() or strafe() commands.
        if (!drivetrain.isBusy()) {
            drivetrain.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
        }
        // If the drivetrain IS busy, the call to drivetrain.drive() is skipped,
        // allowing the path follower in drivetrain.update() to have full control.


        // --- Odometry Reset ---
        if (gamepad1.startWasPressed()) {
            drivetrain.resetOdometry();
        }
    }

    private void updateTelemetry() {
        // Display the robot's current position from odometry
        Pose2D pose = drivetrain.getPose();
        String posStr = String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", posStr);

        // Display whether the drivetrain is busy
        telemetry.addData("Drivetrain Busy", drivetrain.isBusy());

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure all motors are stopped when the OpMode ends.
        drivetrain.stop();
    }
}
