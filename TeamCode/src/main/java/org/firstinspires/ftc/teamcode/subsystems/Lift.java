package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.LiftArm;
import org.opencv.core.Mat;

//Hardware
public class Lift {

    private final LiftArm liftController;

    public static final int POSITION_BOTTOM = 0;
    public static final int POSITION_MIDDLE = 1350;
    public static final int POSITION_TOP = 2690;
    public static final int POSITION_TOLERANCE = 0;

    private LiftState currentLiftState = LiftState.BOTTOM;
    private LiftState previousLiftState = LiftState.BOTTOM;
    private int targetPosition = 0;
    private double manualPower = 0;
    private boolean isManual = true;

    public enum LiftState {
        BOTTOM,
        TOP,
        MIDDLE,
        MANUAL
    }

    public Lift(RobotHardware robot) {

        DcMotorEx liftMotor1 = robot.liftMotor1;
        DcMotorEx liftMotor2 = robot.liftMotor2;
        DigitalChannel liftLimitSwitch = robot.liftLimitSwitch;

        this.liftController = new LiftArm(liftMotor1, liftMotor2, liftLimitSwitch);
    }

    public void update() {
        if (liftController.isLimitPressed() && Math.abs(liftController.getCurrentPosition()) > 10) {
            liftController.resetEncoders();
        }
        if (currentLiftState != previousLiftState) {
            switch (currentLiftState) {
                case TOP:
                    liftController.setTargetPosition(POSITION_TOP);
                    isManual = false;
                    break;
                case BOTTOM:
                    liftController.setTargetPosition(POSITION_BOTTOM);
                    isManual = false;
                    break;
                case MIDDLE:
                    liftController.setTargetPosition(POSITION_MIDDLE);
                    isManual = false;
                    break;
                default:
                    liftController.setTargetPosition(POSITION_BOTTOM);
                    break;
            }
        }
        previousLiftState = currentLiftState;

        if (currentLiftState == LiftState.MANUAL) {
            liftController.setPower(manualPower);
        }
    }

    public void setManualControl(double power) {
        this.manualPower = power;

        if (Math.abs(power) > 0.05) {
            this.currentLiftState = LiftState.MANUAL;
        }
    }

    public void setState(LiftState state) {
        this.currentLiftState = state;

    }


}
