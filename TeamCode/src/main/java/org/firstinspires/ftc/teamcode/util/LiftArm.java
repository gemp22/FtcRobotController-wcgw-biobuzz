package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


public class LiftArm {

    //Hardware
    private final DcMotorEx liftMotor1;
    private final DcMotorEx liftMotor2;

    private final TouchSensor liftLimitSwitch;

    private static final int MAX_HEIGHT = 2690;
    private static final int MIN_HEIGHT = 0;

    private static final double LIFT_SPEED = 1.0;

    public LiftArm(DcMotorEx liftMotor1, DcMotorEx liftMotor2, TouchSensor liftLimitSwitch) {
        this.liftMotor1 = liftMotor1;
        this.liftMotor2 = liftMotor2;
        this.liftLimitSwitch = liftLimitSwitch;

        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setTargetPosition(int target) {
        if (target > MAX_HEIGHT) target = MAX_HEIGHT;
        if (target < MIN_HEIGHT) target = MIN_HEIGHT;

        liftMotor1.setTargetPosition(target);
        liftMotor2.setTargetPosition(target);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(LIFT_SPEED);
        liftMotor2.setPower(LIFT_SPEED);
    }

    public void setPower(double power) {

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (!isLimitPressed() || power < 0) {

            int currentPos = getLiftPosition();
            if (currentPos >= MAX_HEIGHT && power < 0) power = 0;

            liftMotor1.setPower(power);
            liftMotor2.setPower(power);
        }
        else {
            liftMotor1.setPower(0);
            liftMotor2.setPower(0);
        }
    }

    public boolean isLimitPressed() {
        return liftLimitSwitch.isPressed();
    }

    public int getLiftPosition() {
        int currentPosition = liftMotor1.getCurrentPosition();
        return -currentPosition;
    }

    public void resetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
