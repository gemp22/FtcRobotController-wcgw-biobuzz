package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit; // Added for getting current
import org.firstinspires.ftc.teamcode.util.UdpClientPlot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;

public class FlyWheel {
    // --- DEBUGGING ---
    private boolean isDebugEnabled = false;
    private UdpClientPlot plotClient;
    private String plotClientHost = "192.168.43.100";
    private int plotClientPort = 7778;

    // --- Constants ---
    public static double TICKS_PER_REVOLUTION = 28.0; // **MUST BE SET FOR YOUR MOTOR**

    // Default PIDF Coefficients
//    private static final double DEFAULT_KP = 0.03000;
    private static final double DEFAULT_KP = 0.01000;

//    private static final double DEFAULT_KI = 0.00400;
    private static final double DEFAULT_KI = 0.00000;
//    private static final double DEFAULT_KD = 0.00000;
    private static final double DEFAULT_KD = 0.00000;
//    private static final double DEFAULT_KF = 0.0;
    private static final double DEFAULT_KF = 0.5;

    // PIDF Coefficients
    private double kP, kI, kD, kF;

    // --- Hardware ---
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    // --- PID Control Variables ---
    private double targetRPM = 0.0;
    private double targetTicksPerSecond = 0.0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double MAX_INTEGRAL_SUM = 250;
    private static final double INTEGRAL_RESET_THRESHOLD_RPM = 50;

    /**
     * Main constructor for the FlyWheel class.
     */
    public FlyWheel(DcMotorEx motor1, DcMotorEx motor2) {
        this(motor1, motor2, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF);
    }

    /**
     * Constructor with specified PIDF coefficients.
     */
    public FlyWheel(DcMotorEx motor1, DcMotorEx motor2, double kP, double kI, double kD, double kF) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        setPIDFCoefficients(kP, kI, kD, kF);

        // Configure motors
        this.motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor1.setDirection(DcMotor.Direction.REVERSE);
        this.motor2.setDirection(DcMotor.Direction.REVERSE);
//        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidTimer.reset();
    }

    /**
     * Enables or disables the UDP plot client for debugging.
     * Can be called at any time.
     * @param enable True to enable debugging, false to disable.
     */
    public void setDebug(boolean enable) {
        if (enable && !this.isDebugEnabled) {
            // Turning debugging ON
            this.isDebugEnabled = true;
            if (plotClient == null) {
                plotClient = new UdpClientPlot(plotClientHost, plotClientPort);
                if (plotClient.isInitialized()) {
                    long initTime = System.currentTimeMillis();
                    // Setup Left Y-Axis (Y1) for RPM
                    plotClient.sendYLimits(initTime, 6500, 0);
                    plotClient.sendYUnits(initTime + 1, "RPMs");

                    // *** NEW: Setup Right Y-Axis (Y2) for Current in Amps ***
                    plotClient.sendYLimits2(initTime + 2, 10.0, 0.0); // Set Y2 axis from 0 to 2 Amps
                    plotClient.sendYUnits2(initTime + 3, "Amps");    // Set Y2 axis label

                    // Left Y-Axis Series
                    plotClient.sendSeriesNameLine(initTime + 4, "Target RPM", 1);
                    plotClient.sendSeriesNameLine(initTime + 5, "Current RPM", 2);

                    // Right Y-Axis Series
                    plotClient.sendSeriesNameLine2(initTime + 6, "Motor 1 Amps", 3);
                    plotClient.sendSeriesNameLine2(initTime + 7, "Motor 2 Amps", 4);

                    plotClient.sendKeyValue(initTime + 4, "TICKS_PER_REV", String.format(Locale.US, "%.2f", TICKS_PER_REVOLUTION));
                    // *** NEW: Add a marker to show when debug was enabled ***
                    plotClient.sendTextMarker(initTime + 5, "FlyWheel Debug Enabled", "top");
                }
            }
        } else if (!enable && this.isDebugEnabled) {
            // Turning debugging OFF
            this.isDebugEnabled = false;
            close(); // Close the client when disabling
        }
    }

    /**
     * Sets the PIDF coefficients for the flywheel controller.
     */
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        boolean changed = false;
        if (Math.abs(this.kP - kP) > 1e-9) { this.kP = kP; changed = true; }
        if (Math.abs(this.kI - kI) > 1e-9) { this.kI = kI; changed = true; }
        if (Math.abs(this.kD - kD) > 1e-9) { this.kD = kD; changed = true; }
        if (Math.abs(this.kF - kF) > 1e-9) { this.kF = kF; changed = true; }

        if (changed) {
            this.integralSum = 0;
            this.lastError = 0;
            this.pidTimer.reset();

            if (isDebugEnabled && plotClient != null) {
                long time = System.currentTimeMillis();
                plotClient.sendKeyValue(time, "PID_P", String.format(Locale.US, "%.5f", this.kP));
                plotClient.sendKeyValue(time, "PID_I", String.format(Locale.US, "%.5f", this.kI));
                plotClient.sendKeyValue(time, "PID_D", String.format(Locale.US, "%.5f", this.kD));
                plotClient.sendKeyValue(time, "PID_F", String.format(Locale.US, "%.5f", this.kF));
            }
        }
    }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getkF() { return kF; }
    public void setMotor2Direction(DcMotorSimple.Direction direction) {
        if (this.motor2 != null) {
            this.motor2.setDirection(direction);
        }
    }
    public void reset() {
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    public void setRPM(double rpm) {
        if (Math.abs(rpm - this.targetRPM) > INTEGRAL_RESET_THRESHOLD_RPM) {
            reset();
        }
        this.targetRPM = rpm;
        this.targetTicksPerSecond = (rpm * TICKS_PER_REVOLUTION) / 60.0;
        if (rpm == 0) {
            motor1.setPower(0);
            motor2.setPower(0);
            integralSum = 0;
            lastError = 0;
        }
    }
    public double getTargetRPM() {
        return this.targetRPM;
    }
    public double getCurrentRPM() {
        double currentTicksPerSecond = motor2.getVelocity();
        return (currentTicksPerSecond * 60.0) / TICKS_PER_REVOLUTION;
    }

    /**
     * Updates the flywheel motors to try and achieve the target RPM.
     */
    public void update() {
        if (targetRPM == 0) {
            if (motor1.getPower() != 0 || motor2.getPower() != 0) {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            if (isDebugEnabled && plotClient != null) {
                long time = System.currentTimeMillis();
                plotClient.sendLineY(time, targetRPM, 1);
                plotClient.sendLineY(time, getCurrentRPM(), 2);
                plotClient.sendKeyValue(time, "motorPower", "0.00");
                // *** NEW: Send zero current when stopped ***
                plotClient.sendLineY2(time, 0.0, 3); // Motor 1 current
                plotClient.sendLineY2(time, 0.0, 4); // Motor 2 current
            }

            return;
        }

        double currentVelocityTicksPerSecond = motor2.getVelocity();
        double error = targetTicksPerSecond - currentVelocityTicksPerSecond;
        double deltaTime = pidTimer.seconds();
        pidTimer.reset();

        // Reset the integral sum if the error crosses zero.
        // This prevents the "unwinding" process after an overshoot, allowing for much faster settling.
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            integralSum = 0;
//            if (isDebugEnabled && plotClient != null) {
//                // Add a marker to the plot to visualize when the integral is reset
//                plotClient.sendTextMarker(System.currentTimeMillis(), "Integral Reset", "bottom");
//            }
        }

        double p = kP * error;
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL_SUM / kI, MAX_INTEGRAL_SUM / kI);
        double i = kI * integralSum;
        double derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
        double d = kD * derivative;
        lastError = error;

        double motorPower = p + i + d + kF;
        double motorPowerBeforeClip = motorPower;
        motorPower = Range.clip(motorPower, (targetRPM >= 0) ? 0.0 : -1.0, (targetRPM >= 0) ? 1.0 : 0.0);

        motor1.setPower(motorPower);
        motor2.setPower(motorPower);


        if (isDebugEnabled && plotClient != null) {
            long time = System.currentTimeMillis();
            plotClient.sendLineY(time, targetRPM, 1);
            plotClient.sendLineY(time, getCurrentRPM(), 2);

            // *** NEW: Get and plot motor current on the right Y-axis (Y2) ***
            double motor1Amps = motor1.getCurrent(CurrentUnit.AMPS);
            double motor2Amps = motor2.getCurrent(CurrentUnit.AMPS);
            plotClient.sendLineY2(time, motor1Amps, 3); // Style 3 for Motor 1 Current
            plotClient.sendLineY2(time, motor2Amps, 4); // Style 4 for Motor 2 Current
            // Add current values to the key-value table for easy reading
            plotClient.sendKeyValue(time, "motor1_Amps", String.format(Locale.US, "%.3f", motor1Amps));
            plotClient.sendKeyValue(time, "motor2_Amps", String.format(Locale.US, "%.3f", motor2Amps));


            plotClient.sendKeyValue(time, "motorPower", String.format(Locale.US, "%.3f", motorPower));
            plotClient.sendKeyValue(time, "motorPowerBeforeClip", String.format(Locale.US, "%.3f", motorPowerBeforeClip));
            plotClient.sendKeyValue(time, "error", String.format(Locale.US, "%.3f", error));
            plotClient.sendKeyValue(time, "p", String.format(Locale.US, "%.3f", p));
            plotClient.sendKeyValue(time, "i", String.format(Locale.US, "%.3f", i));
            plotClient.sendKeyValue(time, "d", String.format(Locale.US, "%.3f", d));
        }
    }

    /**
     * Call this method to stop the flywheel and reset PID errors.
     */
    public void stop() {
        setRPM(0);

        if (isDebugEnabled && plotClient != null) {
            long time = System.currentTimeMillis();
            plotClient.sendTextMarker(time, "FlyWheel Stopped", "mid");
            plotClient.sendLineY(time, 0, 1);
            plotClient.sendLineY(time, 0, 2);
            // *** NEW: Send zero current on stop ***
            plotClient.sendLineY2(time, 0.0, 3);
            plotClient.sendLineY2(time, 0.0, 4);
            plotClient.sendKeyValue(time, "motorPower", "0.00");
        }
    }

    /**
     * Closes any open resources, like the UDP client.
     */
    public void close() {
        if (plotClient != null) {
            plotClient.close();
            plotClient = null;
        }
        this.isDebugEnabled = false;
    }
}
