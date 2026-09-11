package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.LiftArm;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import java.util.Locale;

/**
 * A TeleOp OpMode for live-tuning the Lift subsystem's PIDF coefficients.
 * Focuses purely on chassis driving and lift configuration variables.
 *
 * --- CONTROLS (Gamepad 1 - Driver) ---
 * Left Stick Y/X:   Drive / Strafe
 * Right Stick X:    Turn
 * Cross Button (A): Go to BOTTOM Position Preset
 * Circle Button (B):Go to MIDDLE Position Preset
 * Triangle Button(Y):Go to TOP Position Preset
 * Square Button (X):Reset Lift operation directly to MANUAL mode
 *
 * --- CONTROLS (Gamepad 2 - Tuner) ---
 * Left Stick Y:     Direct manual open-loop lift power override
 * D-Pad Up/Down:    Tune Proportional Gain (kP) (+ / -)
 * D-Pad Right/Left: Tune Integral Gain (kI) (+ / -)
 * Circle / Cross:   Tune Derivative Gain (kD) (+ / -)
 * L / R Bumpers:    Tune Feedforward Gravity Gain (kF) (+ / -)
 */
@TeleOp(name = "Lift PID Tuner", group = "Test")
public class Lift_PID_Tuner extends OpMode {

    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Lift lift;

    private final MovingStatistics loopTimes = new MovingStatistics(100);
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Tuning Increments
    private static final double P_INCREMENT = 0.0001;
    private static final double I_INCREMENT = 0.00001;
    private static final double D_INCREMENT = 0.00001;
    private static final double F_INCREMENT = 0.001;

    // Local Tuning Registers
    private double currentP = 0.005;
    private double currentI = 0.000;
    private double currentD = 0.000;
    private double currentF = 0.000;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        drivetrain = new Drivetrain(robot);
        lift = new Lift(robot);

        // Fetch initialized default values directly from the core configuration layout layers
        LiftArm arm = lift.getController();
        currentP = arm.getkP();
        currentI = arm.getkI();
        currentD = arm.getkD();
        currentF = arm.getkF();

        telemetry.addLine("Lift PID Tuner OpMode Initialized.");
        telemetry.addLine("G1: Drive & Position Presets | G2: Adjust PIDF Gains & Manual Power");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        // Clear Lynx Bulk cache modules once per cycle loop framework frame iteration
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        // --- GAMEPAD 1: Driver Drivetrain Controls & Preset Commands ---
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

        if (gamepad1.a) {
            lift.setState(Lift.LiftState.BOTTOM);
        } else if (gamepad1.b) {
            lift.setState(Lift.LiftState.MIDDLE);
        } else if (gamepad1.y) {
            lift.setState(Lift.LiftState.TOP);
        } else if (gamepad1.x) {
            lift.setState(Lift.LiftState.MANUAL);
        }

        // --- GAMEPAD 2: Tuner Controls ---
        // Connect direct manual stick override axis
        lift.setManualControl(-gamepad2.left_stick_y);

        // Live parameter tuning loops via active button down ticks checks
        if (gamepad2.dpad_up)    currentP += P_INCREMENT;
        if (gamepad2.dpad_down)  currentP -= P_INCREMENT;
        if (gamepad2.dpad_right) currentI += I_INCREMENT;
        if (gamepad2.dpad_left)  currentI -= I_INCREMENT;
        if (gamepad2.b)          currentD += D_INCREMENT;
        if (gamepad2.a)          currentD -= D_INCREMENT;
        if (gamepad2.right_bumper) currentF += F_INCREMENT;
        if (gamepad2.left_bumper)  currentF -= F_INCREMENT;

        // Force positive range floor clamps on coefficients to preserve system control bounds
        if (currentP < 0) currentP = 0;
        if (currentI < 0) currentI = 0;
        if (currentD < 0) currentD = 0;
        if (currentF < 0) currentF = 0;

        // Continuously stream the latest updated coefficients down into the feedback handler class
        LiftArm arm = lift.getController();
        arm.setPIDFCoefficients(currentP, currentI, currentD, currentF);

        // --- Periodic Subsystem Executions ---
        drivetrain.update();
        lift.update();

        // --- Live Logging Pipelines ---
        updateOpModeTelemetry(arm);

        loopTimes.add(loopTimer.nanoseconds());
    }

    /**
     * Groups structural diagnostic operational metrics for realtime displays on the Driver Station.
     * @param arm Hardware layer reference controller.
     */
    private void updateOpModeTelemetry(LiftArm arm) {
        telemetry.addData("Loop Rate (Hz)", "%.1f", 1 / (loopTimes.getMean() / 1e9));
        
        telemetry.addLine("\n--- Active Live PIDF Gains ---");
        telemetry.addLine(String.format(Locale.US, "kP: %.5f | kI: %.5f | kD: %.5f | kF: %.4f",
                arm.getkP(), arm.getkI(), arm.getkD(), arm.getkF()));

        telemetry.addLine("\n--- Position Metrics (Ticks) ---");
        telemetry.addData("Target Goal Position", arm.getTargetPosition());
        telemetry.addData("Current Height Position", arm.getLiftPosition());
        telemetry.addData("Remaining Error Track", arm.getTargetPosition() - arm.getLiftPosition());
        
        telemetry.addLine("\n--- Hardware Sensor/Input States ---");
        telemetry.addData("Limit Switch Contacted", lift.isLimitPressed());
        telemetry.addData("Stick Manual Power Cmd", lift.manualPower);

        telemetry.addLine("\n--- INTERACTIVE CONTROLS MENU ---");
        telemetry.addLine("G1 Driving: Left Stick = Translate | Right Stick = Turn");
        telemetry.addLine("G1 Presets: Cross: BOTTOM | Circle: MIDDLE | Triangle: TOP | Square: MANUAL Mode");
        telemetry.addLine("G2 Tuning:  DPAD U/D = kP | DPAD R/L = kI | Circle/Cross = kD | Bumpers = kF");
        telemetry.addLine("G2 Override:Left Stick Y = Open-Loop Manual Power Command Override");

        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}
