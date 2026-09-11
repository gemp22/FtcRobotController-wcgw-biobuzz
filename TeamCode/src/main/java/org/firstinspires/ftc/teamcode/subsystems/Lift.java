package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.LiftArm;

/**
 * High-level subsystem representing the Robot Lift.
 * Coordinates automatic height presets and manual joystick control states,
 * and handles automatic encoder re-zeroing via a physical limit switch sensor.
 */
public class Lift {

    private final LiftArm liftController;

    // --- Height Presets (Encoder Ticks) ---
    public static final int POSITION_BOTTOM = 0;
    public static final int POSITION_MIDDLE = 1350;
    public static final int POSITION_TOP = 2690;
    public static final int POSITION_TOLERANCE = 25;

    // --- State Variables ---
    public boolean isLimitPressed = false;
    private LiftState currentLiftState = LiftState.BOTTOM;
    private LiftState previousLiftState = LiftState.BOTTOM;
    public double manualPower = 0;

    /**
     * Enumeration representing the control modes / location states of the Lift.
     */
    public enum LiftState {
        BOTTOM,
        TOP,
        MIDDLE,
        MANUAL
    }

    /**
     * Constructs the Lift subsystem using mapped hardware device channels.
     * @param robot Global configuration hub containing raw hardware references.
     */
    public Lift(RobotHardware robot) {
        DcMotorEx liftMotor1 = robot.liftMotor1;
        DcMotorEx liftMotor2 = robot.liftMotor2;
        TouchSensor liftLimitSwitch = robot.liftLimitSwitch;

        this.liftController = new LiftArm(liftMotor1, liftMotor2, liftLimitSwitch);
    }

    /**
     * Periodic evaluation execution cycle loop called once per OpMode framework iteration.
     * Manages sensor checks, target height updates, and delegates control to the hardware driver.
     */
    public void update() {
        // Evaluate limit switch boundary contact to trigger immediate single-shot encoder re-calibration
        if (liftController.isLimitPressed() && !isLimitPressed) {
            isLimitPressed = true;
            liftController.resetEncoders();
        } else if (!liftController.isLimitPressed()) {
            isLimitPressed = false;
        }

        // Handle one-time preset position changes when the structural state has shifted
        if (currentLiftState != previousLiftState) {
            switch (currentLiftState) {
                case TOP:
                    liftController.setTargetPosition(POSITION_TOP);
                    break;
                case BOTTOM:
                    liftController.setTargetPosition(POSITION_BOTTOM);
                    break;
                case MIDDLE:
                    liftController.setTargetPosition(POSITION_MIDDLE);
                    break;
                default:
                    liftController.setTargetPosition(POSITION_BOTTOM);
                    break;
            }
        }
        previousLiftState = currentLiftState;

        // Continuously pump user joystick inputs if manual override control is fully engaged
        if (currentLiftState == LiftState.MANUAL) {
            liftController.setPower(manualPower);
        }
        
        // Execute underlying closed-loop PID controllers or manual hardware updates
        liftController.update();
    }

    /**
     * Returns the normalized current vertical height position computed by the hardware layer.
     * @return Position value expressed in encoder ticks.
     */
    public double getLiftHeight() {
        return liftController.getLiftPosition();
    }

    /**
     * Evaluates if the bottom-most touch limit switch sensor is physically pressed.
     * @return True if button is triggered, false otherwise.
     */
    public boolean isLimitPressed() {
        return liftController.isLimitPressed();
    }

    /**
     * Sets open-loop manual stick driving power commands, switching control modes automatically if beyond threshold.
     * @param power Power scalar clamped from -1.0 (downwards) to 1.0 (upwards).
     */
    public void setManualControl(double power) {
        this.manualPower = power;

        // Deadband threshold checks to cleanly hook stick activation overrides
        if (Math.abs(power) > 0.05) {
            this.currentLiftState = LiftState.MANUAL;
        }
    }

    /**
     * Forcefully changes the active operating location state configuration of the lift.
     * @param state The desired target LiftState preset configuration.
     */
    public void setState(LiftState state) {
        this.currentLiftState = state;
    }

    /**
     * Logs comprehensive operational metadata and states onto the Driver Station interface.
     * @param telemetry The framework Driver Station logging interface hook.
     */
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Lift State", currentLiftState);
        telemetry.addData("Lift Height (Ticks)", getLiftHeight());
        telemetry.addData("Limit Switch Pressed", isLimitPressed());
        telemetry.addData("Manual Power Cmd", manualPower);
    }

    public LiftArm getController() {
        return liftController;
    }
}
