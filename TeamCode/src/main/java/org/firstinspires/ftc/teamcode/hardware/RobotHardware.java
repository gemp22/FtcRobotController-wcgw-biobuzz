package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver; // Assuming this is the correct package for GoBildaPinpointDriver
import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.List;

/**
 * This class centralizes the hardware mapping and configuration for the robot.
 * Its purpose is to:
 * 1.  Define all hardware configuration names in one place (as public static final strings).
 * 2.  Declare all hardware device objects.
 * 3.  Provide a single init() method that performs all hardwareMap.get() calls.
 *
 * This makes the robot's configuration easier to manage and decouples the subsystems
 * from the hardware mapping process.
 */
public class RobotHardware {

    // --- HARDWARE NAME CONSTANTS ---
    // These names must match the configuration on the Driver Station/Control Hub.

    // Drivetrain
    public static final String FRONT_LEFT_DRIVE_NAME = "front_left_drive";
    public static final String BACK_LEFT_DRIVE_NAME = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE_NAME = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE_NAME = "back_right_drive";
    public static final String ODO_NAME = "odo";

    public static final String INTAKE_ROLLER_NAME = "intake_roller_1";

    public static final String INTAKE_ROLLER_2_NAME = "intake_roller_2";
//    // Intake & Ball Transport
//    public static final String INTAKE_ROLLER_NAME = "intake_roller";
//    public static final String ROLLER_BED_MOTOR_NAME = "turret_encoder"; // we use the same port for the turret encoder and the roller bed motor
////    public static final String LEFT_ROLLER_BED_SERVO_NAME = "left_roller_bed_servo";
////    public static final String RIGHT_ROLLER_BED_SERVO_NAME = "right_roller_bed_servo";
//    public static final String LEFT_GATE_SERVO_NAME = "left_gate_servo";
//    public static final String MID_GATE_SERVO_NAME = "mid_gate_servo";
//    public static final String RIGHT_GATE_SERVO_NAME = "right_gate_servo";
//    public static final String LEFT_FEEDER_SERVO_NAME = "left_feeder_servo";
//    public static final String RIGHT_FEEDER_SERVO_NAME = "right_feeder_servo";
//    public static final String LEFT_BOOSTER_SERVO_NAME = "left_booster_servo";
//    public static final String RIGHT_BOOSTER_SERVO_NAME = "right_booster_servo";
//    public static final String INTAKE_CAMERA_NAME = "huskylens";
//    public static final String LIFTER_SERVO_NAME = "lifter_servo";
//
//    // Color Sensors
//    public static final String LEFT_COLOR_SENSOR_NAME = "left_color_sensor";
//    public static final String MID_COLOR_SENSOR_NAME = "mid_color_sensor";
//    public static final String RIGHT_COLOR_SENSOR_NAME = "right_color_sensor";
//
//    // Turret & Shooter
////    public static final String TURRET_SERVO_NAME = "turret_servo";
////    public static final String TURRET_ENCODER_NAME = "turret_encoder";
////    public static final String SHOOTER_SERVO_NAME = "shooter_servo";
////    public static final String FLYWHEEL_1_NAME = "flywheel_1";
////    public static final String FLYWHEEL_2_NAME = "flywheel_2";
////    public static final String LIMIT_SWITCH_LEFT_NAME = "limit_switch_left";
////    public static final String LIMIT_SWITCH_RIGHT_NAME = "limit_switch_right";
////    public static final String SHOOTER_BEAM_SENSOR_1_NAME = "shooter_beam_sensor_1";
////    public static final String SHOOTER_BEAM_SENSOR_2_NAME = "shooter_beam_sensor_2";
////    public static final String LIMELIGHT_NAME = "limelight";
////    public static final String TURRET_LED_1_GREEN_NAME = "turret_led_1_green"; // Vision Status LED
////    public static final String TURRET_LED_1_RED_NAME = "turret_led_1_red";
////    public static final String TURRET_LED_2_GREEN_NAME = "turret_led_2_green"; // Odometry Status LED
////    public static final String TURRET_LED_2_RED_NAME = "turret_led_2_red";


    // --- HARDWARE DEVICE OBJECTS ---

    // Drivetrain
    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public GoBildaPinpointDriver odo;

    // Intake & Ball Transport
//    public DcMotor intakeRoller;
//    public DcMotor rollerBedMotor;
//    public CRServo leftRollerBedServo;
//    public CRServo rightRollerBedServo;
//    public Servo leftGateServo;
//    public Servo midGateServo;
//    public Servo rightGateServo;
//    public CRServo leftFeederServo;
//    public CRServo rightFeederServo;
//    public CRServo leftBoosterServo;
//    public CRServo rightBoosterServo;
//    public HuskyLens huskyLens;
    public DcMotorEx intakeRoller1;

    public DcMotorEx intakeRoller2;

    // Color Sensors
//    public NormalizedColorSensor leftCS;
//    public NormalizedColorSensor midCS;
//    public NormalizedColorSensor rightCS;

    // Turret & Shooter
//    public CRServo turretServo;
//    public DcMotorEx turretEncoder;
//    public Servo shooterServo;
//    public DcMotorEx flywheelMotor1;
//    public DcMotorEx flywheelMotor2;
//    public DigitalChannel limitSwitchLeft;
//    public DigitalChannel limitSwitchRight;
//    public Limelight3A limelight;
//    public DigitalChannel shooterBeamSensor1;
//    public DigitalChannel shooterBeamSensor2;
//    public DigitalChannel turretLed1Green;
//    public DigitalChannel turretLed1Red;
//    public DigitalChannel turretLed2Green;
//    public DigitalChannel turretLed2Red;
//    public Servo lifterServo;
    public List<LynxModule> allHubs;

    /**
     * Initializes all hardware devices from the given HardwareMap.
     * This method should be called once in the init() of your OpMode.
     * @param hardwareMap The HardwareMap from the OpMode, used to map strings to devices.
     */
    public void init(HardwareMap hardwareMap) {
        // Drivetrain
        frontLeftDrive = hardwareMap.get(DcMotor.class, FRONT_LEFT_DRIVE_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BACK_LEFT_DRIVE_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FRONT_RIGHT_DRIVE_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BACK_RIGHT_DRIVE_NAME);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, ODO_NAME);
        intakeRoller1 = hardwareMap.get(DcMotorEx.class, INTAKE_ROLLER_NAME);
        intakeRoller2 = hardwareMap.get(DcMotorEx.class, INTAKE_ROLLER_2_NAME);

        // Intake & Ball Transport
//        intakeRoller = hardwareMap.get(DcMotor.class, INTAKE_ROLLER_NAME);
//        rollerBedMotor = hardwareMap.get(DcMotor.class, ROLLER_BED_MOTOR_NAME);
////        leftRollerBedServo = hardwareMap.get(CRServo.class, LEFT_ROLLER_BED_SERVO_NAME);
////        rightRollerBedServo = hardwareMap.get(CRServo.class, RIGHT_ROLLER_BED_SERVO_NAME);
//        leftGateServo = hardwareMap.get(Servo.class, LEFT_GATE_SERVO_NAME);
//        midGateServo = hardwareMap.get(Servo.class, MID_GATE_SERVO_NAME);
//        rightGateServo = hardwareMap.get(Servo.class, RIGHT_GATE_SERVO_NAME);
//        leftFeederServo = hardwareMap.get(CRServo.class, LEFT_FEEDER_SERVO_NAME);
//        rightFeederServo = hardwareMap.get(CRServo.class, RIGHT_FEEDER_SERVO_NAME);
//        leftBoosterServo = hardwareMap.get(CRServo.class, LEFT_BOOSTER_SERVO_NAME);
//        rightBoosterServo = hardwareMap.get(CRServo.class, RIGHT_BOOSTER_SERVO_NAME);
//        huskyLens = hardwareMap.get(HuskyLens.class, INTAKE_CAMERA_NAME);

        // Color Sensors
//        leftCS = hardwareMap.get(NormalizedColorSensor.class, LEFT_COLOR_SENSOR_NAME);
//        midCS = hardwareMap.get(NormalizedColorSensor.class, MID_COLOR_SENSOR_NAME);
//        rightCS = hardwareMap.get(NormalizedColorSensor.class, RIGHT_COLOR_SENSOR_NAME);

        // Turret & Shooter
//        turretServo = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);
//        turretEncoder = hardwareMap.get(DcMotorEx.class, TURRET_ENCODER_NAME);
//        shooterServo = hardwareMap.get(Servo.class, SHOOTER_SERVO_NAME);
//        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, FLYWHEEL_1_NAME);
//        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, FLYWHEEL_2_NAME);
//        limitSwitchLeft = hardwareMap.get(DigitalChannel.class, LIMIT_SWITCH_LEFT_NAME);
//        limitSwitchRight = hardwareMap.get(DigitalChannel.class, LIMIT_SWITCH_RIGHT_NAME);
//        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
//        shooterBeamSensor1 = hardwareMap.get(DigitalChannel.class, SHOOTER_BEAM_SENSOR_1_NAME);
//        shooterBeamSensor2 = hardwareMap.get(DigitalChannel.class, SHOOTER_BEAM_SENSOR_2_NAME);
//        turretLed1Green = hardwareMap.get(DigitalChannel.class, TURRET_LED_1_GREEN_NAME);
//        turretLed1Red = hardwareMap.get(DigitalChannel.class, TURRET_LED_1_RED_NAME);
//        turretLed2Green = hardwareMap.get(DigitalChannel.class, TURRET_LED_2_GREEN_NAME);
//        turretLed2Red = hardwareMap.get(DigitalChannel.class, TURRET_LED_2_RED_NAME);
//        lifterServo = hardwareMap.get(Servo.class, LIFTER_SERVO_NAME);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

//    public void initFlywheel(HardwareMap hardwareMap) {
//        // Drivetrain
//        frontLeftDrive = hardwareMap.get(DcMotor.class, FRONT_LEFT_DRIVE_NAME);
//        backLeftDrive = hardwareMap.get(DcMotor.class, BACK_LEFT_DRIVE_NAME);
//        frontRightDrive = hardwareMap.get(DcMotor.class, FRONT_RIGHT_DRIVE_NAME);
//        backRightDrive = hardwareMap.get(DcMotor.class, BACK_RIGHT_DRIVE_NAME);
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, ODO_NAME);
//
//        // Intake & Ball Transport
//        intakeRoller = hardwareMap.get(DcMotor.class, INTAKE_ROLLER_NAME);
//        rollerBedMotor = hardwareMap.get(DcMotor.class, ROLLER_BED_MOTOR_NAME);
////        leftRollerBedServo = hardwareMap.get(CRServo.class, LEFT_ROLLER_BED_SERVO_NAME);
////        rightRollerBedServo = hardwareMap.get(CRServo.class, RIGHT_ROLLER_BED_SERVO_NAME);
//        leftGateServo = hardwareMap.get(Servo.class, LEFT_GATE_SERVO_NAME);
//        midGateServo = hardwareMap.get(Servo.class, MID_GATE_SERVO_NAME);
//        rightGateServo = hardwareMap.get(Servo.class, RIGHT_GATE_SERVO_NAME);
//        leftFeederServo = hardwareMap.get(CRServo.class, LEFT_FEEDER_SERVO_NAME);
//        rightFeederServo = hardwareMap.get(CRServo.class, RIGHT_FEEDER_SERVO_NAME);
//        leftBoosterServo = hardwareMap.get(CRServo.class, LEFT_BOOSTER_SERVO_NAME);
//        rightBoosterServo = hardwareMap.get(CRServo.class, RIGHT_BOOSTER_SERVO_NAME);
//        huskyLens = hardwareMap.get(HuskyLens.class, INTAKE_CAMERA_NAME);
//
//        // Color Sensors
//        leftCS = hardwareMap.get(NormalizedColorSensor.class, LEFT_COLOR_SENSOR_NAME);
//        midCS = hardwareMap.get(NormalizedColorSensor.class, MID_COLOR_SENSOR_NAME);
//        rightCS = hardwareMap.get(NormalizedColorSensor.class, RIGHT_COLOR_SENSOR_NAME);
//
//        // Turret & Shooter
//        turretServo = hardwareMap.get(CRServo.class, TURRET_SERVO_NAME);
//        turretEncoder = hardwareMap.get(DcMotorEx.class, TURRET_ENCODER_NAME);
//        shooterServo = hardwareMap.get(Servo.class, SHOOTER_SERVO_NAME);
//        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, FLYWHEEL_1_NAME);
//        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, FLYWHEEL_2_NAME);
////        limitSwitchLeft = hardwareMap.get(DigitalChannel.class, LIMIT_SWITCH_LEFT_NAME);
////        limitSwitchRight = hardwareMap.get(DigitalChannel.class, LIMIT_SWITCH_RIGHT_NAME);
////        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
//        shooterBeamSensor1 = hardwareMap.get(DigitalChannel.class, SHOOTER_BEAM_SENSOR_1_NAME);
//        shooterBeamSensor2 = hardwareMap.get(DigitalChannel.class, SHOOTER_BEAM_SENSOR_2_NAME);
////        turretLed1Green = hardwareMap.get(DigitalChannel.class, TURRET_LED_1_GREEN_NAME);
////        turretLed1Red = hardwareMap.get(DigitalChannel.class, TURRET_LED_1_RED_NAME);
////        turretLed2Green = hardwareMap.get(DigitalChannel.class, TURRET_LED_2_GREEN_NAME);
//        turretLed2Red = hardwareMap.get(DigitalChannel.class, TURRET_LED_2_RED_NAME);
//        lifterServo = hardwareMap.get(Servo.class, LIFTER_SERVO_NAME);

//    }
}
