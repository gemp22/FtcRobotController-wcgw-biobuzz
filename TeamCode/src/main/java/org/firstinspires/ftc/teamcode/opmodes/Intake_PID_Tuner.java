package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeRoller;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import java.util.Locale;

/**
 * A TeleOp for live-tuning the Intake's PIDF coefficients.
 *
 * --- CONTROLS (Gamepad 1 - Driver) ---
 * Left Stick: Drive/Strafe | Right Stick: Turn
 * B Button: Intake FORWARD | X Button: Intake REVERSE | Y Button: Intake OFF
 *
 * --- CONTROLS (Gamepad 2 - Tuner) ---
 * D-Pad Up/Down:    Tune P
 * D-Pad Right/Left: Tune I
 * B / A Buttons:    Tune D
 * L / R Bumpers:    Tune F
 */
@TeleOp(name = "Intake PID Tuner", group = "Test")
public class Intake_PID_Tuner extends OpMode {

    private RobotHardware robot;
    private Intake intake;
    private Drivetrain drivetrain;

    private MovingStatistics loopTimes = new MovingStatistics(100);
    private ElapsedTime loopTimer = new ElapsedTime();

    // Tuning increments
    private static final double P_INCREMENT = 0.0001;
    private static final double I_INCREMENT = 0.0001;
    private static final double D_INCREMENT = 0.00005;
    private static final double F_INCREMENT = 0.005;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        intake = new Intake(robot);
        drivetrain = new Drivetrain(robot);
        
        telemetry.addLine("Intake PID Tuner Initialized.");
        telemetry.addLine("G1: Drive & Intake | G2: Tune PIDF");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        // G1: Driving & Intake Toggles (Matching TeleOp Main)
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
        
        if (gamepad1.b) intake.changeState(Intake.IntakeState.FORWARD);
        if (gamepad1.x) intake.changeState(Intake.IntakeState.REVERSE);
        if (gamepad1.y) intake.changeState(Intake.IntakeState.OFF);

        // G2: PIDF Tuning
        IntakeRoller roller = intake.getRollerController();
        
        // P Tuning
        if (gamepad2.dpad_up)    roller.adjustP(P_INCREMENT);
        if (gamepad2.dpad_down)  roller.adjustP(-P_INCREMENT);
        
        // I Tuning
        if (gamepad2.dpad_right) roller.adjustI(I_INCREMENT);
        if (gamepad2.dpad_left)  roller.adjustI(-I_INCREMENT);
        
        // D Tuning
        if (gamepad2.b)          roller.adjustD(D_INCREMENT);
        if (gamepad2.a)          roller.adjustD(-D_INCREMENT);
        
        // F Tuning
        if (gamepad2.right_bumper) roller.adjustF(F_INCREMENT);
        if (gamepad2.left_bumper)  roller.adjustF(-F_INCREMENT);

        // Subsystem Heartbeats
        drivetrain.update();
        intake.update();

        // Telemetry Update
        updateTelemetry();

        loopTimes.add(loopTimer.nanoseconds());
    }

    private void updateTelemetry() {
        IntakeRoller roller = intake.getRollerController();

        telemetry.addData("Loop (Hz)", "%.1f", 1 / (loopTimes.getMean() / 1e9));
        telemetry.addLine(String.format(Locale.US, "P:%.5f I:%.5f D:%.5f F:%.5f", 
                roller.getkP(), roller.getkI(), roller.getkD(), roller.getkF()));
        telemetry.addData("RPM", "Tgt:%.0f | Act:%.0f", roller.getTargetRPM(), roller.getCurrentRPM());
        telemetry.addData("State", intake.getIntakeState().toString());

        telemetry.addLine("\n--- CONTROLS ---");
        telemetry.addLine("G1: B:FWD | X:REV | Y:OFF");
        telemetry.addLine("G2: DPAD U/D:P | DPAD R/L:I");
        telemetry.addLine("G2: B/A:D | Bumpers:F");
        
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
    }
}
