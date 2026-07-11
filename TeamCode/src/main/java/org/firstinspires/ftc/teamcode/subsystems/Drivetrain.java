package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.pathing.PathFollower;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Locale; // Import Locale for formatting

import java.util.ArrayList;

/**
 * The Drivetrain subsystem is responsible for all robot motion, including
 * Mecanum driving, odometry position tracking, and autonomous path following.
 */
public class Drivetrain {

    // --- Constants ---
    private static final double STRAFE_GAIN = 1.3; // To counteract friction in strafing
    private final double GAIN_INCREMENT = 0.1;
    private static final double AUTONOMOUS_PATH_TOLERANCE = 1; // The robot is considered "at the point" if it's within 1 inch
    private static final double AUTONOMOUS_MOVE_SPEED = 0.8;
    private static final double AUTONOMOUS_TURN_SPEED = 0.5;
    // --- Lifter servo constants ---
    private static final double LIFTER_UP_POSITION = 0.16;   // Example value, to be calibrated
    private static final double LIFTER_DOWN_POSITION = 0.38; // Example value, to be calibrated
    private static final double LIFTER_INCREMENT = 0.02;  // For manual calibration

    private static final double RED_GOAL_WALL_ANGLE_RAD = Math.toRadians(36.0);
    private static final double BLUE_GOAL_WALL_ANGLE_RAD = Math.toRadians(-36.0);

    // --- Hardware ---
    private final DcMotor frontLeftDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backRightDrive;
    private final GoBildaPinpointDriver odo;
    private final PathFollower pathFollower;
//    private final Servo lifterServo;

    // --- State ---
    private double backWheelGain = 1.0;
    private Pose2D cachedPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES, 0);
    private Velocity cachedLinearVelocity = new Velocity(DistanceUnit.INCH, 0, 0, 0, 0);
    private double cachedAngularVelocityRad = 0.0;

    // --- Autonomous Path Following State ---
    private ArrayList<CurvePoint> currentPath = null;
    private double pathFollowAngle;
    private boolean isDebugMode = false;

    /**
     * Constructor for the Drivetrain subsystem.
     * @param robot The centralized hardware hub.
     */
    public Drivetrain(RobotHardware robot) {
        // Assign hardware from the hardware hub
        this.frontLeftDrive = robot.frontLeftDrive;
        this.backLeftDrive = robot.backLeftDrive;
        this.frontRightDrive = robot.frontRightDrive;
        this.backRightDrive = robot.backRightDrive;
        this.odo = robot.odo;
        this.pathFollower = new PathFollower();
//        this.lifterServo = robot.lifterServo;

        // --- INITIAL CONFIGURATION ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the lifter to the default 'up' position on init
//        setLifterUp();

        // Odometry configuration
        odo.setOffsets(171.45, -117.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // Important: Robot must be sationaty at this point to calibrate the IMU.
//        odo.recalibrateIMU();
    }

    /**
     * Drives the robot using robot-centric Mecanum style inputs.
     * @param y  The forward/backward power (-1.0 to 1.0).
     * @param x  The strafing power (-1.0 to 1.0).
     * @param rx The turning power (-1.0 to 1.0).
     */
    public void drive(double y, double x, double rx) {
        // --- MODIFIED: Apply the strafe gain to the x input ---
        double x_with_gain = x * STRAFE_GAIN;

        // The Mecanum drive logic from your original OpMode.
        // The gain is applied directly to the back wheels.
        double denominator = Math.max(Math.abs(y) + Math.abs(x_with_gain) + Math.abs(rx), 1);
        frontLeftDrive.setPower((y + x_with_gain - rx) / denominator);
        backLeftDrive.setPower((y - x_with_gain - rx) / denominator * backWheelGain);
        frontRightDrive.setPower((y - x_with_gain + rx) / denominator);
        backRightDrive.setPower((y + x_with_gain + rx) / denominator * backWheelGain);

    }

    /**
     * Stops all drivetrain motors.
     */
    public void stop() {
        // If we were following a path, clear it.
        currentPath = null;
        drive(0, 0, 0);
    }

    /**
     * Updates the odometry position tracking and handles autonomous path following.
     * This should be called in every loop of the OpMode.
     */
    public void update() {
        // ALWAYS update odometry
        odo.update();

        // Cache the pose ONCE per loop
        this.cachedPose = odo.getPosition();
        this.cachedLinearVelocity = new Velocity(DistanceUnit.INCH, odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), 0, System.nanoTime());
        this.cachedAngularVelocityRad = Math.toRadians(odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

        // If we are currently following a path, execute the path following logic.
        if (isBusy()) {
            // Get the robot's current pose and velocity from our odometry
            Pose2D currentPose = getPose();
            Velocity currentLinVel = getLinearVelocity();
            double currentAngVel = getAngularVelocity();

            // Call the static PathFollower utility to get the desired motor powers
            PathFollower.PathingResult result = this.pathFollower.followCurve(
                    currentPath,
                    currentPose,
                    currentLinVel,
                    currentAngVel,
                    pathFollowAngle,
                    AUTONOMOUS_PATH_TOLERANCE,
                    this.isDebugMode
            );

            // Set the motor powers based on the pure pursuit calculation.
            // IMPORTANT: PathFollower returns robot-centric powers, so we pass them to our robot-centric drive method.
            drive(result.moveY, result.moveX, result.turn);

            // If the path is finished, clear the current path to stop following
            if (result.isFinished) {
                pathFollower.reset();
                stop(); // This will clear the path and stop motors
            }
        }
    }

    /**
     * Enables or disables path following debug visuals for the simulator.
     * @param enable True to enable, false to disable.
     */
    public void setDebug(boolean enable) {
        if (pathFollower != null) {
            pathFollower.setDebug(enable);
        }
    }

    /**
     * Closes any open debugging resources. Should be called in OpMode's stop().
     */
    public void closeDebug() {
        if (pathFollower != null) {
            pathFollower.close();
        }
    }


    /**
     * Drives the robot forward or backward a specific distance from its current location.
     * The robot will face forward along the direction of travel.
     * @param inches The distance in inches to drive. Positive is forward, negative is backward.
     */
    public void driveForward(double inches) {
        Pose2D currentPose = getPose();
        double heading = currentPose.getHeading(AngleUnit.RADIANS);

        // Calculate the target coordinates based on current position, heading, and distance.
        double targetX = currentPose.getX(DistanceUnit.INCH) + (inches * Math.cos(heading));
        double targetY = currentPose.getY(DistanceUnit.INCH) + (inches * Math.sin(heading));

        // Create the path
        ArrayList<CurvePoint> pathToTarget = new ArrayList<>();
        pathToTarget.add(new CurvePoint(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH),
                AUTONOMOUS_MOVE_SPEED, AUTONOMOUS_TURN_SPEED, 10.0, Math.toRadians(60), 0.6));
        pathToTarget.add(new CurvePoint(targetX, targetY,
                AUTONOMOUS_MOVE_SPEED, AUTONOMOUS_TURN_SPEED, 10.0, Math.toRadians(60), 0.6));

        // Determine the follow angle based on direction.
        // 90 degrees for forward, -90 for backward.
        double angle = (inches >= 0) ? 90.0 : -90.0;
        followPath(pathToTarget, Math.toRadians(angle), true);
    }

    /**
     * Strafes the robot left or right a specific distance from its current location.
     * The robot's heading will be kept perpendicular to the direction of travel.
     * @param inches The distance in inches to strafe. Positive is left, negative is right.
     */
    public void strafe(double inches) {
        Pose2D currentPose = getPose();
        // A strafe is a movement 90 degrees (PI/2 radians) offset from the current heading.
        double strafeHeading = currentPose.getHeading(AngleUnit.RADIANS) + (Math.PI / 2.0);

        // Calculate the target coordinates.
        double targetX = currentPose.getX(DistanceUnit.INCH) + (inches * Math.cos(strafeHeading));
        double targetY = currentPose.getY(DistanceUnit.INCH) + (inches * Math.sin(strafeHeading));

        // Create the path
        ArrayList<CurvePoint> pathToTarget = new ArrayList<>();
        pathToTarget.add(new CurvePoint(currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH),
                AUTONOMOUS_MOVE_SPEED, AUTONOMOUS_TURN_SPEED, 10.0, Math.toRadians(60), 0.6));
        pathToTarget.add(new CurvePoint(targetX, targetY,
                AUTONOMOUS_MOVE_SPEED, AUTONOMOUS_TURN_SPEED, 10.0, Math.toRadians(60), 0.6));

        // Determine the follow angle based on direction.
        // 0 degrees for strafing right, 180 degrees for strafing left.
        double angle = (inches >= 0) ? 180.0 : 0.0;
        followPath(pathToTarget, Math.toRadians(angle), true);
    }


    /**
     * Instructs the drivetrain to begin following a specified path.
     * @param path The list of CurvePoints that defines the path.
     * @param followAngle The angle (in radians) the robot should maintain relative to the path.
     * @param debug If true, this path will send debug data to the simulator.
     */
    public void followPath(ArrayList<CurvePoint> path, double followAngle, boolean debug) {
        if (path == null || path.isEmpty()) {
            System.err.println("DRIVETRAIN ERROR: followPath called with a null or empty path.");
            return;
        }
        this.currentPath = path;
        this.pathFollowAngle = followAngle;
        this.isDebugMode = debug; // Store the debug flag

        // Log the start of the path following action
        CurvePoint startPoint = path.get(0);
        CurvePoint endPoint = path.get(path.size() - 1);
        System.out.printf(Locale.US, "DRIVETRAIN: Starting new path. Points: %d, Target Angle: %.1f deg, Path Start: (%.1f, %.1f), End: (%.1f, %.1f)%n", path.size(), Math.toDegrees(followAngle), startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }

    /**
     * Calculates the straight-line distance from the robot's current position
     * to the final point of the active path.
     *
     * @return The distance in inches. Returns Double.MAX_VALUE if no path is active.
     */
    public double getDistanceToFinalPoint() {
        // 1. Check if there is a valid path to measure against.
        if (!isBusy() || currentPath.isEmpty()) {
            return Double.MAX_VALUE;
        }

        // 2. Get the coordinates of the last point in the path.
        CurvePoint finalPoint = currentPath.get(currentPath.size() - 1);
        double finalX = finalPoint.x;
        double finalY = finalPoint.y;

        // 3. Get the robot's current coordinates from the cached pose.
        double robotX = cachedPose.getX(DistanceUnit.INCH);
        double robotY = cachedPose.getY(DistanceUnit.INCH);

        // 4. Calculate the hypotenuse (distance) between the two points.
        return Math.hypot(finalX - robotX, finalY - robotY);
    }

    /**
     * Checks if the drivetrain is currently busy executing an autonomous movement.
     * @return True if following a path, false otherwise.
     */
    public boolean isBusy() {
        return currentPath != null;
    }

    /**
     * A special update method that ONLY updates odometry. Used for the test OpMode
     * to prevent the main `update()` from interfering.
     */
    public void update_TEST_ONLY() {
        odo.update();
    }



    // --- Odometry Getter and Setter Methods ---

    /**
     * Sets the robot's current position and heading in the odometry system.
     * This is useful for defining a starting position for field-centric coordinates.
     * @param pos The new Pose2D for the robot.
     */
    public void setPosition(Pose2D pos) {
        odo.setPosition(pos);
        this.cachedPose = pos;
        // Reset velocity caches as they are now invalid
        this.cachedLinearVelocity = new Velocity(DistanceUnit.INCH, 0, 0, 0, 0);
        this.cachedAngularVelocityRad = 0.0;
    }

    /**
     * Resets the robot's position and heading in the odometry system. Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU.
     */
    public void resetOdometry() {
        odo.resetPosAndIMU();
        this.cachedPose = new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES, 0);
        this.cachedLinearVelocity = new Velocity(DistanceUnit.INCH, 0, 0, 0, 0);
        this.cachedAngularVelocityRad = 0.0;
        if (pathFollower != null) {
            pathFollower.reset();
        }
    }

    /**
     * Gets the robot's current position and heading from odometry.
     * @return A Pose2D object representing the robot's current pose.
     */
    public Pose2D getPose() {
        // Return the cached value instead of making a new hardware call
        return this.cachedPose;
    }

    /**
     * Gets the robot's current linear (x, y) velocity from odometry.
     * @return A Velocity object.
     */
    public Velocity getLinearVelocity() {
        return this.cachedLinearVelocity;
        // Construct a Velocity object from the individual components provided by the odometry driver.
//        return new Velocity(DistanceUnit.INCH, odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), 0, System.nanoTime());
    }

    /**
     * Calculates the robot's velocity component parallel to the specified alliance goal wall.
     * This is useful for controllers that need to know how fast the robot is moving alongside the wall.
     *
     * @param alliance The alliance goal wall to use as a reference.
     * @return The velocity in inches per second, parallel to the wall.
     */
    public double getParallelVelocity(Alliance.Color alliance) {
        // 1. Get the robot's current field-centric velocity components.
        double vx = this.cachedLinearVelocity.xVeloc;
        double vy = this.cachedLinearVelocity.yVeloc;


        // 2. Select the angle of the wall (theta) based on the alliance.
        double theta;
        if (alliance == Alliance.Color.BLUE) {
            theta = BLUE_GOAL_WALL_ANGLE_RAD;
        } else { // Assumes RED alliance
            theta = RED_GOAL_WALL_ANGLE_RAD;
        }

        // 3. Project the velocity vector onto the wall vector using the dot product formula.
        // v_parallel = v . u  (where u is the unit vector of the wall direction)
        // v_parallel = (vx * ux) + (vy * uy)
        // Since u = (cos(theta), sin(theta)), the formula is:
        double v_parallel = vx * Math.cos(theta) + vy * Math.sin(theta);

        return v_parallel;
    }

    /**
     * Gets the robot's current angular velocity (turn rate) from odometry.
     * @return Angular velocity in radians per second.
     */
    public double getAngularVelocity() {
        return this.cachedAngularVelocityRad;
//        // Use UnnormalizedAngleUnit as required by the GoBildaPinpointDriver class
//        double degreesPerSecond = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
//        return Math.toRadians(degreesPerSecond);
    }

    /**
     * Gets the robot's current angular velocity (turn rate) from odometry in the specified units.
     * This is a more direct wrapper around the GoBildaPinpointDriver method.
     * @param unit The desired angle unit for the velocity (e.g., AngleUnit.DEGREES).
     * @return The heading velocity in the specified units per second.
     */
    public double getHeadingVelocity(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            return Math.toDegrees(this.cachedAngularVelocityRad);
        } else {
            return this.cachedAngularVelocityRad;
        }
    }

    // --- TeleOp Adjustment Methods ---

    /**
     * Increases the gain for the back wheels.
     */
    public void increaseBackWheelGain() {
        backWheelGain += GAIN_INCREMENT;
        backWheelGain = Range.clip(backWheelGain, 0, 5);
    }

    /**
     * Decreases the gain for the back wheels.
     */
    public void decreaseBackWheelGain() {
        backWheelGain -= GAIN_INCREMENT;
        backWheelGain = Range.clip(backWheelGain, 0, 5);
    }

    /**
     * Gets the current back wheel gain value.
     * @return The current gain.
     */
    public double getBackWheelGain() {
        return this.backWheelGain;
    }

    /**
     * Moves the lifter servo to its predefined 'down' position to lift the robot.
     */
//    public void setLifterDown() {
//        if (lifterServo != null) {
//            lifterServo.setPosition(LIFTER_DOWN_POSITION);
//        }
//    }

    /**
     * Moves the lifter servo to its predefined 'up' position to lower the robot.
     */
//    public void setLifterUp() {
//        if (lifterServo != null) {
//            lifterServo.setPosition(LIFTER_UP_POSITION);
//        }
//    }

    /**
     * Adjusts the lifter servo position incrementally for calibration.
     * @param increase True to move the servo position up (towards 1.0), false to move it down (towards 0.0).
     */
//    public void adjustLifterPosition(boolean increase) {
//        if (lifterServo != null) {
//            double currentPosition = lifterServo.getPosition();
//            double newPosition = currentPosition + (increase ? LIFTER_INCREMENT : -LIFTER_INCREMENT);
//            lifterServo.setPosition(Range.clip(newPosition, 0.16, 1.0));
//        }
//    }

    /**
     * Gets the current position of the lifter servo.
     * @return The servo's current position, or -1 if not available.
     */
//    public double getLifterPosition() {
//        if (lifterServo != null) {
//            return lifterServo.getPosition();
//        }
//        return -1;
//    }

    /**
     * Resets the robot's odometry position using the Limelight's distance to the target.
     * This method assumes the robot is facing directly forward (0-degree heading).
     *
     * @param turret The turret subsystem, to get sensor data from.
     * @return true if the position was successfully reset, false otherwise.
     */
//    public boolean resetPositionWithVision(Turret turret) {
//        // 1. Check for necessary conditions
//        if (!turret.isTargetVisible()) {
//            System.err.println("DRIVETRAIN-RESET: Cannot reset position. Vision target not visible.");
//            return false;
//        }
//
//        Alliance.Color alliance = turret.getTargetAlliance();
//        if (alliance == null) {
//            System.err.println("DRIVETRAIN-RESET: Cannot reset position. No target alliance selected.");
//            return false;
//        }
//
//        // 2. Get the required sensor data
//        double distanceToTarget = turret.getTargetDistanceByAngle(); // Hypotenuse from CAMERA to TAG
//        if (distanceToTarget <= 0) {
//            System.err.println("DRIVETRAIN-RESET: Invalid distance from Limelight.");
//            return false;
//        }
//        double turretAngleDeg = turret.getCurrentAngle();
//
//        // 3. Determine the known absolute coordinates of the AprilTag
//        Pose2D aprilTagPose;
//        if (alliance == Alliance.Color.BLUE) {
//            aprilTagPose = Turret.BLUE_GOAL_POSE;
//        } else {
//            aprilTagPose = Turret.RED_GOAL_POSE;
//        }
//
//        // 4. Perform the trigonometric calculation
//        // Turret angle is also the absolute angle from the CAMERA's perspective, because robot heading is 0.
//        // Convert turret angle to radians. Negate it to match the standard CCW unit circle.
//        double angleFromCameraToTagRad = Math.toRadians(-turretAngleDeg);
//
//        // Calculate the vector from the CAMERA to the TAG
//        double vectorX_camToTag = Math.cos(angleFromCameraToTagRad) * distanceToTarget;
//        double vectorY_camToTag = Math.sin(angleFromCameraToTagRad) * distanceToTarget;
//
//        // 5. Calculate the CAMERA's absolute position on the field
//        // Camera_X = Tag_X - Vector_X
//        double cameraX = aprilTagPose.getX(DistanceUnit.INCH) - vectorX_camToTag;
//        double cameraY = aprilTagPose.getY(DistanceUnit.INCH) - vectorY_camToTag;
//
//        // 6. Calculate the ROBOT's center position by accounting for the camera's forward offset.
//        // Since the robot's heading is 0, the camera is simply offset along the X-axis.
//        // Robot_X = Camera_X - CAMERA_FORWARD_OFFSET
//        double newRobotX = cameraX - Turret.CAMERA_FORWARD_OFFSET_INCHES;
//        double newRobotY = cameraY; // Y position is the same as the camera's Y when heading is 0.
//
//        // 7. Log the change and set the new position
//        Pose2D currentPose = getPose();
//        Pose2D newPose = new Pose2D(DistanceUnit.INCH, newRobotX, newRobotY, AngleUnit.DEGREES, 0.0); // Assume 0 heading
//
//        System.out.printf(Locale.US, "DRIVETRAIN-RESET: Odo reset from (X:%.1f, Y:%.1f, H:%.1f) to (X:%.1f, Y:%.1f, H:%.1f)%n",
//                currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.DEGREES),
//                newPose.getX(DistanceUnit.INCH), newPose.getY(DistanceUnit.INCH), newPose.getHeading(AngleUnit.DEGREES));
//
//        setPosition(newPose);
//        return true;
//    }

//    /**
//     * Resets the robot's odometry position using the Limelight's distance to the target.
//     * This method assumes the robot is facing directly forward (0-degree heading).
//     *
//     * @param turret The turret subsystem, to get sensor data from.
//     */
//    public void resetPositionWithVision(Turret turret) {
//        // 1. Check for necessary conditions
//        if (!turret.isTargetVisible()) {
//            System.err.println("DRIVETRAIN-RESET: Cannot reset position. Vision target not visible.");
//            return;
//        }
//
//        Alliance.Color alliance = turret.getTargetAlliance();
//        if (alliance == null) {
//            System.err.println("DRIVETRAIN-RESET: Cannot reset position. No target alliance selected.");
//            return;
//        }
//
//        // 2. Get the required sensor data
//        double distanceToTarget = turret.getTargetDistanceByAngle(); // Hypotenuse from CAMERA to TAG
//        if (distanceToTarget <= 0) {
//            System.err.println("DRIVETRAIN-RESET: Invalid distance from Limelight.");
//            return;
//        }
//        double turretAngleDeg = turret.getCurrentAngle();
//
//        // 3. Determine the known absolute coordinates of the AprilTag
//        Pose2D aprilTagPose;
//        if (alliance == Alliance.Color.BLUE) {
//            aprilTagPose = Turret.BLUE_GOAL_POSE;
//        } else {
//            aprilTagPose = Turret.RED_GOAL_POSE;
//        }
//
//        // 4. Perform the trigonometric calculation
//        // Turret angle is also the absolute angle from the CAMERA's perspective, because robot heading is 0.
//        // Convert turret angle to radians. Negate it to match the standard CCW unit circle.
//        double angleFromCameraToTagRad = Math.toRadians(-turretAngleDeg);
//
//        // Calculate the vector from the CAMERA to the TAG
//        double vectorX_camToTag = Math.cos(angleFromCameraToTagRad) * distanceToTarget;
//        double vectorY_camToTag = Math.sin(angleFromCameraToTagRad) * distanceToTarget;
//
//        // 5. Calculate the CAMERA's absolute position on the field
//        // Camera_X = Tag_X - Vector_X
//        double cameraX = aprilTagPose.getX(DistanceUnit.INCH) - vectorX_camToTag;
//        double cameraY = aprilTagPose.getY(DistanceUnit.INCH) - vectorY_camToTag;
//
//        // --- THIS IS THE FIX ---
//        // 6. Calculate the ROBOT's center position by accounting for the camera's forward offset.
//        // Since the robot's heading is 0, the camera is simply offset along the X-axis.
//        // Robot_X = Camera_X - CAMERA_FORWARD_OFFSET
//        double newRobotX = cameraX - Turret.CAMERA_FORWARD_OFFSET_INCHES;
//        double newRobotY = cameraY; // Y position is the same as the camera's Y when heading is 0.
//
//        // 7. Log the change and set the new position
//        Pose2D currentPose = getPose();
//        Pose2D newPose = new Pose2D(DistanceUnit.INCH, newRobotX, newRobotY, AngleUnit.DEGREES, 0.0); // Assume 0 heading
//
//        System.out.printf(Locale.US, "DRIVETRAIN-RESET: Odo reset from (X:%.1f, Y:%.1f, H:%.1f) to (X:%.1f, Y:%.1f, H:%.1f)%n",
//                currentPose.getX(DistanceUnit.INCH), currentPose.getY(DistanceUnit.INCH), currentPose.getHeading(AngleUnit.DEGREES),
//                newPose.getX(DistanceUnit.INCH), newPose.getY(DistanceUnit.INCH), newPose.getHeading(AngleUnit.DEGREES));
//
//        setPosition(newPose);
//    }
}
