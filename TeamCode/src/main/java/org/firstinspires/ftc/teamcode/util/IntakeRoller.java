package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * IntakeRoller provides a closed-loop PIDF controller for intake motors.
 *
 * Logic Overview:
 * - Intake (Forward): Uses PIDF to maintain a target RPM despite battery drop or game piece load.
 * - Outtake (Reverse): Bypasses PID math to provide raw 100% power for maximum ejection force.
 */
public class IntakeRoller {
    // --- CONSTANTS ---
    // The GoBILDA 5203 series 6000 RPM motor has 28 ticks per revolution on its internal encoder.
    public static double TICKS_PER_REVOLUTION = 28.0;

    // Default PIDF Coefficients
    private static final double DEFAULT_KP = 0.01; // Adjust if the motor is too slow to react to load
    private static final double DEFAULT_KI = 0.00; // Adjust to eliminate steady-state error
    private static final double DEFAULT_KD = 0.00; // Adjust to dampen oscillations
    private static final double DEFAULT_KF = 0.50; // Feedforward: Baseline power needed to spin at ~50% speed

    // Safety limits for the Integral term to prevent "Windup" (runaway power increase)
    private static final double MAX_INTEGRAL_SUM = 250;
    private static final double INTEGRAL_RESET_THRESHOLD_RPM = 50;

    // --- STATE VARIABLES ---
    private double kP, kI, kD, kF;
    private double targetRPM = 0.0;
    private double targetTicksPerSecond = 0.0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Manual Mode flag used to switch between Closed-Loop (RPM) and Open-Loop (Raw Power)
    private boolean isManualMode = false;
    private double manualPower = 0.0;

    // --- HARDWARE ---
    private final DcMotorEx intakeMotor1;
    private final DcMotorEx intakeMotor2;

    public IntakeRoller(DcMotorEx motor1, DcMotorEx motor2) {
        this(motor1, motor2, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF);
    }

    public IntakeRoller(DcMotorEx motor1, DcMotorEx motor2, double kP, double kI, double kD, double kF) {
        this.intakeMotor1 = motor1;
        this.intakeMotor2 = motor2;

        setPIDFCoefficients(kP, kI, kD, kF);

        // Hardware Configuration:
        // 1. RUN_WITHOUT_ENCODER allows us to use our own PID math rather than the built-in SDK logic.
        this.intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 2. Directions are set so positive power pulls game pieces into the robot.
        this.intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        this.intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        // 3. ZeroPowerBehavior.FLOAT allows the rollers to spin freely when power is 0.
        this.intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidTimer.reset();
    }

    /**
     * Updates the PIDF gains. Resets the internal state to prevent erratic behavior after change.
     */
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF;
        reset();
    }

    /**
     * Clears error history and resets the loop timer.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    /**
     * Sets the target speed for the intake. Switching to RPM disables manual power mode.
     * @param rpm Target revolutions per minute (e.g., 3000 for 50% speed).
     */
    public void setRPM(double rpm) {
        isManualMode = false;
        // If the target changes drastically, reset I to prevent overshoot
        if (Math.abs(rpm - this.targetRPM) > INTEGRAL_RESET_THRESHOLD_RPM) {
            reset();
        }
        this.targetRPM = rpm;
        this.targetTicksPerSecond = (rpm * TICKS_PER_REVOLUTION) / 60.0;
    }

    /**
     * Sets a direct power level (Open-Loop). Enables Manual Mode, bypassing PID math.
     * @param power Power level from -1.0 to 1.0.
     */
    public void setPower(double power) {
        isManualMode = true;
        manualPower = power;
    }

    // Getters for Telemetry
    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }

    // Live tuning adjustments
    public void adjustP(double delta) { this.kP += delta; reset(); }
    public void adjustI(double delta) { this.kI += delta; reset(); }
    public void adjustD(double delta) { this.kD += delta; reset(); }
    public void adjustF(double delta) { this.kF += delta; reset(); }

    public double getCurrentRPM() {
        double currentTPS = intakeMotor1.getVelocity();
        return (currentTPS * 60.0) / TICKS_PER_REVOLUTION;
    }

    public double getTargetRPM() {
        return targetRPM;
    }


    /**
     * Main control loop. Calculates error and sets motor power.
     * MUST be called every loop in the subsystem update() method.
     */
    public void update() {
        // --- OPEN LOOP MODE ---
        if (isManualMode) {
            intakeMotor1.setPower(manualPower);
            intakeMotor2.setPower(manualPower);
            return;
        }

        // --- CLOSED LOOP MODE ---
        if (targetRPM == 0) {
            intakeMotor1.setPower(0);
            intakeMotor2.setPower(0);
            return;
        }

        // 1. Measure current speed (Ticks per second)
        double currentVelocityTPS = intakeMotor2.getVelocity();

        // 2. Calculate error (Target vs Actual)
        double error = targetTicksPerSecond - currentVelocityTPS;
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // 3. Anti-Windup Logic: Reset I if the error crosses zero to prevent settling oscillation.
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integralSum = 0;
        }

        // 4. Calculate PID Terms
        double p = kP * error;

        integralSum += error * deltaTime;
        if (kI != 0) {
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL_SUM / kI, MAX_INTEGRAL_SUM / kI);
        }
        double i = kI * integralSum;

        double d = kD * ((deltaTime > 0) ? (error - lastError) / deltaTime : 0);
        lastError = error;

        // 5. Final Power Calculation (PID + Feedforward)
        // Clip power based on target direction to ensure it only spins the intended way.
        double motorPower = Range.clip(p + i + d + kF, (targetRPM >= 0) ? 0.0 : -1.0, (targetRPM >= 0) ? 1.0 : 0.0);

        intakeMotor1.setPower(motorPower);
        intakeMotor2.setPower(motorPower);
    }

    /**
     * E-Stop for the intake. Stops motors and clears internal control states.
     */
    public void stop() {
        setRPM(0);
        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
        reset();
    }
}