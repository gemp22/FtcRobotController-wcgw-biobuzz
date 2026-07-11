package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(name = "Test: MoveToPoint", group = "Tests")
@Disabled
public class Auto_TestMoveToPoint extends OpMode {

    private RobotHardware robot;
    private Drivetrain drivetrain;

    private CurvePoint targetPoint = new CurvePoint(50, 0, 0.4, 0.4, 10, Math.toRadians(60), 0.6);
    private double targetAngle = Math.toRadians(75);

    private ElapsedTime timer = new ElapsedTime();
    private boolean isFinished;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // 1. Re-instantiate hardware and subsystems to ensure a clean state.
        robot = new RobotHardware();
        robot.init(hardwareMap);
        drivetrain = new Drivetrain(robot);

        // 2. THIS IS THE CRITICAL FIX for the second run failing.
        //    We must re-enable debug mode on every init() call.
        drivetrain.setDebug(true);

        // 3. Reset hardware odometry and set our logical starting position.
        drivetrain.resetOdometry();
        drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // 4. Reset OpMode state variables.
        isFinished = false;

        telemetry.addData("Status", "Ready. Target is (50, 0)");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (isFinished) {
            drivetrain.stop(); // Ensure motors are off
            telemetry.addData("Status", "Finished!");
            telemetry.update();
            return;
        }

        // Call the test method in the drivetrain
//        boolean atTarget = drivetrain.moveToPoint_TEST(targetPoint, targetAngle);

        // IMPORTANT: The Drivetrain's own update method must still be called
        // to handle odometry updates. We use the test-only version.
        drivetrain.update_TEST_ONLY();

//        if (atTarget || timer.seconds() > 10) {
//            isFinished = true;
//            drivetrain.stop();
//        }

        // Telemetry
        Pose2D currentPose = drivetrain.getPose();
        telemetry.addData("X (in)", "%.2f", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.2f", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.2f", currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up resources, especially the debug client.
        if (drivetrain != null) {
            drivetrain.closeDebug();
        }
        super.stop();
    }
}
