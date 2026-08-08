package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BallColor;

/**
 * The Intake subsystem is responsible for collecting balls from the field,
 * transporting them internally, detecting their color, and managing the gates
 * that release them to the shooter.
 */
public class Intake {

    // --- Constants ---
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

    // --- State ---


    private IntakeState currentIntakeState = IntakeState.OFF;

    private IntakeState previousIntakeState = IntakeState.OFF;


    // --- Flag to control color sensor updates ---private boolean isColorSensingEnabled = false;

    // --- Flag to control HuskyLens camera updates ---

    // --- State variables to track last set power ---
    private double lastIntakeRollerPower = -999;
    // State variables for individual gate toggles

    public Intake(RobotHardware robot) {
        // Assign hardware from the hub
        this.intakeRoller = robot.intakeRoller;


        // Instantiate helper objects

        // --- INITIAL CONFIGURATION ---
        this.intakeRoller.setDirection(DcMotor.Direction.REVERSE);

        // Set initial positions and powers
        this.intakeRoller.setPower(0);

        }

    // --- High-Level Control Methods ---







    /**
     * Directly sets the state of the main intake roller.
     * @param on True to turn the roller on, false to turn it off.
     */





    /**
     * This method should be called in every loop of the OpMode.
     * It handles state updates like timed gate closing and setting motor powers.
     */
    public void update() {
        // --- Only update camera if enabled ---

        // --- Only set power if it has changed ---

        // 1. Intake Roller


        if (currentIntakeState != previousIntakeState) {
            switch (currentIntakeState) {
                case FORWARD:
                    intakeRoller.setPower(INTAKE_ROLLER_SPEED_FORWARD);
                    break;
                case REVERSE:
                    intakeRoller.setPower(INTAKE_ROLLER_SPEED_BACKWARD);
                    break;
                case OFF:
                    intakeRoller.setPower(0);
                default:
                    break;
            }
            previousIntakeState = currentIntakeState;
        }
        // 2. Roller Bed Motor


        // 3. Feeder Servos
        // Determine the target power for each feeder servo

        // Set left feeder power if it changed

    }


    /**
     * Stops all motors and servos AND resets the state of the subsystem.
     */
    public void stop() {
        // --- 1. Set all hardware to a stopped state ---
        intakeRoller.setPower(0);


        // --- 2. Reset all state variables to their default values ---
        changeState(IntakeState.OFF);

        // --- 3. Close any open resources ---
        // This ensures the debug plotters are closed correctly
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
}
