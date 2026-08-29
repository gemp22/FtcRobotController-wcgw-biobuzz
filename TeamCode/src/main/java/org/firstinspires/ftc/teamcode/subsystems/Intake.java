package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.IntakeRoller;


/**
 * The Intake subsystem is responsible for collecting balls from the field,
 * transporting them internally, detecting their color, and managing the gates
 * that release them to the shooter.
 */
public class Intake {

    // --- Constants ---
    private final IntakeRoller rollerController;
    private static final double INTAKE_ROLLER_SPEED = 1.0;

    private static final double INTAKE_ROLLER_SPEED_FORWARD = 1.0;
    private static final double INTAKE_ROLLER_SPEED_BACKWARD = -1.0;

    private static final double CRSERVO_FORWARD_POWER = 1.0;
    private static final double CRSERVO_REVERSE_POWER = -1.0;
    private static final double CRSERVO_STOP_POWER = 0.0;
    private static final double ROLLER_BED_SPEED_INCREMENT = 0.1;
    private static final long GATE_OPEN_DURATION_MS = 3000;

    // --- Constants for Shake Mode ---
    private static final long ROLLER_BED_SHAKE_FORWARD_MS = 300;
    private static final long ROLLER_BED_SHAKE_REVERSE_MS = 400;
    private static final double ROLLER_BED_SHAKE_SPEED = 0.7;

    private static final double LEFT_GATE_OPEN_POS = 1;
    private static final double LEFT_GATE_CLOSE_POS = 0.279;
    private static final double MID_GATE_OPEN_POS = 0;
    private static final double MID_GATE_CLOSE_POS = .743;
    private static final double RIGHT_GATE_OPEN_POS = 0;
    private static final double RIGHT_GATE_CLOSE_POS = 0.74;
    // --- enum for the Intake states ---
    public enum IntakeState {
        OFF,
        FORWARD,
        REVERSE
    }

    // --- Hardware ---
    private final DcMotor intakeRoller;
    private final DcMotor intakeRoller2;

    // --- State ---


    private IntakeState currentIntakeState = IntakeState.OFF;

    private IntakeState previousIntakeState = IntakeState.OFF;


    // --- Flag to control color sensor updates ---private boolean isColorSensingEnabled = false;

    // --- Flag to control HuskyLens camera updates ---

    // --- State variables to track last set power ---
    private double lastIntakeRollerPower = -999;

    public Intake(RobotHardware robot) {
        this.intakeRoller = robot.intakeRoller;
        this.intakeRoller2 = robot.intakeRoller2;

        // NEW: Cast motors to DcMotorEx to support velocity-based PIDF control
        DcMotorEx motor1 = (DcMotorEx) robot.intakeRoller;
        DcMotorEx motor2 = (DcMotorEx) robot.intakeRoller2;

        // NEW: Initialize the PIDF controller utility
        this.rollerController = new IntakeRoller(motor1, motor2);

        // NEW: Set PIDF coefficients (kP, kI, kD, kF)
        // 0.5 kF provides ~50% baseline power for the 6000 RPM motors
        this.rollerController.setPIDFCoefficients(0.01, 0.0, 0.0, 0.5);

        /* --- OLD CODE (PRE-PIDF) ---
        this.intakeRoller.setDirection(DcMotor.Direction.FORWARD);
        this.intakeRoller2.setDirection(DcMotor.Direction.FORWARD);
        this.intakeRoller.setPower(0);
        this.intakeRoller2.setPower(0);
        */
    }

    // --- High-Level Control Methods ---







    /**
     * Directly sets the state of the main intake roller.
     * @param on True to turn the roller on, false to turn it off.
     */





    /**
     * This method handles state updates for the intake roller.
     * Uses the PIDF controller for forward motion and manual power for reverse.
     */
    public void update() {
        if (currentIntakeState != previousIntakeState) {
            switch (currentIntakeState) {
                case FORWARD:
                    // NEW: Use PIDF to maintain 3000 RPM (approx 50% speed)
                    rollerController.setRPM(3000);
                    /* --- OLD CODE ---
                    intakeRoller.setPower(INTAKE_ROLLER_SPEED_FORWARD);
                    intakeRoller2.setPower(INTAKE_ROLLER_SPEED_FORWARD);
                    */
                    break;
                case REVERSE:
                    // NEW: Use manual override for 100% outtake power
                    rollerController.setPower(-1.0);
                    /* --- OLD CODE ---
                    intakeRoller.setPower(INTAKE_ROLLER_SPEED_BACKWARD);
                    intakeRoller2.setPower(INTAKE_ROLLER_SPEED_BACKWARD);
                    */
                    break;
                case OFF:
                    // NEW: Use controller to stop and reset PID internal state
                    rollerController.stop();
                    /* --- OLD CODE ---
                    intakeRoller.setPower(0);
                    intakeRoller2.setPower(0);
                    */
                    break;
                default:
                    rollerController.stop();
                    break;
            }
            previousIntakeState = currentIntakeState;
        }

        // NEW: Heartbeat - calculates and applies motor power every loop
        rollerController.update();
    }


    /**
     * Stops all motors and resets the subsystem state.
     */
    public void stop() {
        // NEW: Stop the PID controller (handles resetting motor power to 0)
        rollerController.stop();
        
        /* --- OLD CODE ---
        intakeRoller.setPower(0);
        intakeRoller2.setPower(0);
        */

        changeState(IntakeState.OFF);
    }

    /**
     * Enables debug plotting for a specific color sensor.
     * Disables debugging on all others.
     * @param sensorIndex 0 for Left, 1 for Mid, 2 for Right. Use -1 to disable all.
     */
    public void setDebug(int sensorIndex) {
        }


    // --- Private Helper Methods ---

    /**
     * Manages the state machine for the roller bed shake mode.
     * Should only be called from update() when isShakeModeOn is true.
     */


    // --- Telemetry Methods ---

    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    /**
     * Gets the current state of the feeders.
     * @return The FeederState enum (OFF, FORWARD, or REVERSE).
     */

    public void changeState(IntakeState newState) {
        currentIntakeState = newState;

    }

    public IntakeRoller getRollerController() {
        return rollerController;
    }
}
