package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Underlying low-level hardware controller driver object for the dual-motor linear lift assembly.
 * Manages raw motor outputs, mathematical tracking logic, structural safety limits,
 * and encapsulates a complete time-delta based closed-loop custom PIDF positioning controller.
 */
public class LiftArm {

    // --- Hardware Access Hooks ---
    private final DcMotorEx liftMotor1;
    private final DcMotorEx liftMotor2;
    private final TouchSensor liftLimitSwitch;

    // --- Hardcoded Structural Safety Limits (Ticks) ---
    private static final int MAX_HEIGHT = 2690;
    private static final int MIN_HEIGHT = 0;
    private static final double LIFT_SPEED = 1.0;

    // --- Default PIDF Tuning Gains ---
    private static final double DEFAULT_KP = 0.0021;
    private static final double DEFAULT_KI = 0.00025;
    private static final double DEFAULT_KD = 0.00005;
    private static final double DEFAULT_KF = 0.000;

    // --- Advanced Integral Stability Constraints ---
    private static final double MAX_INTEGRAL_SUM = 250;
    private static final double INTEGRAL_RESET_THRESHOLD_TICKS = 100;

    // --- PID Control Loop Registers ---
    private double kP, kI, kD, kF;
    private int targetPosition = 0;
    private double lastError = 0;
    private double integralSum = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // --- Open Loop Manual State Hooks ---
    private boolean isManualMode = true;
    private double manualPower = 0.0;

    /**
     * Constructs and initializes the lift core hardware devices, sets rotational parity directions,
     * resets encoder counts, and engages holding brakes to protect against gravity drift.
     * @param liftMotor1 Core encoder-tracked high-stage master lift motor.
     * @param liftMotor2 Paired dual-stage follower lift motor.
     * @param liftLimitSwitch Zero-point validation hardware tactile limit sensor.
     */
    public LiftArm(DcMotorEx liftMotor1, DcMotorEx liftMotor2, TouchSensor liftLimitSwitch) {
        this.liftMotor1 = liftMotor1;
        this.liftMotor2 = liftMotor2;
        this.liftLimitSwitch = liftLimitSwitch;

        // Synchronize opposite-facing physical orientations to match uniform vertical tracking
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flash encoder registers back down cleanly to zero points
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Enforce direct power scaling tracking modes to allow custom software loop control overrides
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turn on aggressive static holding resistance blocks to counteract passive carriage drop forces
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setPIDFCoefficients(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF);
    }

    /**
     * Changes the internal automatic closed-loop targeting tracking position coordinate point.
     * Clamps values safely within structural boundaries and resets loop integrals when switching spans.
     * @param target Desired destination height coordinate expressed in positive encoder ticks.
     */
    public void setTargetPosition(int target) {
        int boundedTarget = target;
        if (boundedTarget > MAX_HEIGHT) boundedTarget = MAX_HEIGHT;
        if (boundedTarget < MIN_HEIGHT) boundedTarget = MIN_HEIGHT;

        isManualMode = false;
        // Wipe historical error accumulators if changing targets sharply to suppress inductive path kicks
        if (Math.abs(boundedTarget - this.targetPosition) > INTEGRAL_RESET_THRESHOLD_TICKS) {
            reset();
        }
        this.targetPosition = boundedTarget;
    }

    /**
     * Modifies the operational tuning coefficient variables governing active closed-loop calculation cycles.
     * Automatically clears working loops if parameters shift during active routines.
     * @param p Proportional response gain multiplier.
     * @param i Integral historical steady-state error gain multiplier.
     * @param d Derivative dampening velocity prediction filter gain multiplier.
     * @param f Constant static feedforward gravity resistance offset power.
     */
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        if (Math.abs(this.kP - p) > 1e-9 || Math.abs(this.kI - i) > 1e-9 ||
            Math.abs(this.kD - d) > 1e-9 || Math.abs(this.kF - f) > 1e-9) {
            
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.kF = f;
            reset();
        }
    }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }
    public int getTargetPosition() { return targetPosition; }

    /**
     * Sets open-loop raw voltage scaling factors, immediately killing background closed-loop tracking.
     * @param power Direct power command from -1.0 (downwards) to 1.0 (upwards).
     */
    public void setPower(double power) {
        isManualMode = true;
        manualPower = power;
    }

    /**
     * Evaluates the electronic voltage activation level of the structural reference hardware button.
     * @return True if carriage is contacting physical bottom, false otherwise.
     */
    public boolean isLimitPressed() {
        return liftLimitSwitch.isPressed();
    }

    /**
     * Collects raw hardware orientation data and applies inversion mapping scales.
     * @return Normalized tracking position where upward displacement scales positively.
     */
    public int getLiftPosition() {
        int currentPosition = liftMotor1.getCurrentPosition();
        return currentPosition;
    }

    /**
     * Resets raw core encoder ticks cleanly to zero points while preserving working state tracking parameters.
     */
    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Flushes internal operational state registers clean to completely erase tracking histories.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    /**
     * Evaluates structural tracking conditions and routes calculated operational powers down to hardware hooks.
     * Runs continuously inside the master iterative scheduling frames.
     */
    public void update() {
        int currentPos = getLiftPosition();

        // --- OPEN LOOP MODE ---
        if (isManualMode) {
            applyPower(manualPower, currentPos);
            return;
        }

        // --- CLOSED LOOP MODE ---
        double error = targetPosition - currentPos;
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Flush integral sums immediately on target crossovers to combat overshoot tendencies
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integralSum = 0;
        }

        // Calculate Proportional component
        double p = kP * error;

        // Calculate and clamp Integral history component to defeat windup saturation
        integralSum += error * deltaTime;
        if (kI != 0) {
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL_SUM / kI, MAX_INTEGRAL_SUM / kI);
        }
        double i = kI * integralSum;

        // Calculate Derivative component using change velocity over delta time
        double d = kD * ((deltaTime > 0) ? (error - lastError) / deltaTime : 0);
        lastError = error;

        // Superimpose all analytical feedback channels alongside baseline gravity-fighting Feedforward blocks
        double motorPower = p + i + d + kF;
        motorPower = Range.clip(motorPower, -1.0, 1.0);

        applyPower(motorPower, currentPos);
    }

    /**
     * Sifts power requests through strict software and electronic physical switch safety corridors.
     * Blocks destructive hardware collisions before issuing adjustments to actual motor outputs.
     * @param power Checked candidate power profile command.
     * @param currentPos Current calculated vertical height layout tick level.
     */
    private void applyPower(double power, int currentPos) {
        double safePower = power;

        // Electronic Hardware Safety: Block dangerous downwards driving forces if structural limit switch is pressed
        if (isLimitPressed() && safePower < 0) {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
            return;
        }

        // Software Position Boundaries Safety: Clamp directional inputs to zero if exceeding structural boundaries
        if (currentPos >= MAX_HEIGHT && safePower > 0) safePower = 0;
        if (currentPos <= MIN_HEIGHT && safePower < 0) safePower = 0;

        liftMotor1.setPower(safePower);
        liftMotor2.setPower(safePower);
    }

    /**
     * Analyzes if the tracking carriage position resides within acceptable convergence tolerances.
     * @return True if structural convergence has stabilized close to goal, false otherwise.
     */
    public boolean atTarget() {
        return Math.abs(targetPosition - getLiftPosition()) <= 25;
    }

    /**
     * Outputs raw analytical values generated inside the loop down to live driver tracking dashboard streams.
     * @param telemetry Active operational telemetry dashboard pipeline.
     */
    public void addPIDTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("PID Target", targetPosition);
        telemetry.addData("PID Current Pos", getLiftPosition());
        telemetry.addData("PID Error", targetPosition - getLiftPosition());
        telemetry.addData("PID Integral Sum", integralSum);
        telemetry.addData("PID Last Error", lastError);
    }
}
