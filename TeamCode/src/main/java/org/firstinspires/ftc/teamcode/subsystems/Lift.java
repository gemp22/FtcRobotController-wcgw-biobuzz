package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.LiftArm;

//Hardware
public class Lift {

    private final LiftArm liftController;

    public Lift(RobotHardware robot) {


        DcMotorEx liftMotor1 = robot.liftMotor1;
        DcMotorEx liftMotor2 = robot.liftMotor2;


        this.liftController = new LiftArm(liftMotor1, liftMotor2);
    }
}
