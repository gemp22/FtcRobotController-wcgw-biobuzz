package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;




public class LiftArm {

    //Hardware
    private final DcMotorEx liftMotor1;
    private final DcMotorEx liftMotor2;

    public LiftArm(DcMotorEx liftMotor1, DcMotorEx liftMotor2) {
        this.liftMotor1 = liftMotor1;
        this.liftMotor2 = liftMotor2;
    }

}
