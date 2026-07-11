package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.UdpClientFieldSim;

import java.util.Locale;

/**
 * The FieldSimulator subsystem is responsible for sending robot state data
 * to a UDP-based field simulator application for visualization and debugging.
 */
public class FieldSimulator {

    private UdpClientFieldSim client;
    private boolean isActive = false;

    // --- AprilTag field coordinates ---
    private static final double RED_APRILTAG_X = 60.0;
    private static final double RED_APRILTAG_Y = -55.0;
    private static final double BLUE_APRILTAG_X = 60.0;
    private static final double BLUE_APRILTAG_Y = 55.0;

    // --- The offset from the robot's center to the limelight ---
    private static final double ROBOT_FRONT_OFFSET_INCHES = 7.0;

    // State tracking to only send data when it changes
    private BallColor.DetectedColor prevLeftColor = BallColor.DetectedColor.NONE;
    private BallColor.DetectedColor prevMidColor = BallColor.DetectedColor.NONE;
    private BallColor.DetectedColor prevRightColor = BallColor.DetectedColor.NONE;
    private double prevBackWheelGain = -1; // Initialize to a value that will always trigger the first send

    public FieldSimulator() {
        // Constructor is empty; initialization happens in setActive()
    }

    /**
     * Activates or deactivates the simulator client.
     * @param active True to activate and connect, false to deactivate and disconnect.
     */
    public void setActive(boolean active) {
        if (active && !isActive) {
            // Activate
            client = new UdpClientFieldSim("192.168.43.100", 7777);
            isActive = true;
        } else if (!active && isActive) {
            // Deactivate
            close();
        }
    }

    /**
     * The main update loop for the simulator. Gathers data from other subsystems
     * and sends it over UDP if the simulator is active.
     * @param drivetrain The drivetrain subsystem to get pose data from.
//     * @param intake     The intake subsystem to get ball color data from.
//     * @param turret     The turret subsystem to get target visibility and mode.
     */
    public void update(Drivetrain drivetrain) {
        if (!isActive || client == null || !client.isInitialized()) {
            return;
        }

        // --- Send Robot Pose ---
        Pose2D currentPose = drivetrain.getPose();
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        double robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS); // Get heading in radians for math
        client.sendPosition(robotX, robotY, currentPose.getHeading(AngleUnit.DEGREES));

        // --- Send Line of Sight and Distance Data ---
//        Turret.TurretMode mode = turret.getMode();
//        boolean isAiming = (mode == Turret.TurretMode.AUTO_AIM || mode == Turret.TurretMode.SEEK_AND_AIM || mode == Turret.TurretMode.ODOMETRY_AIM);

//        if (isAiming) {
//            double tagX = 0;
//            double tagY = 0;
//            boolean targetFound = false;
//
//            Alliance.Color target = turret.getTargetAlliance();
//            if (target == Alliance.Color.BLUE) {
//                tagX = BLUE_APRILTAG_X;
//                tagY = BLUE_APRILTAG_Y;
//                targetFound = true;
//            } else if (target == Alliance.Color.RED) {
//                tagX = RED_APRILTAG_X;
//                tagY = RED_APRILTAG_Y;
//                targetFound = true;
//            }
//
//            if (targetFound) {
//                // --- Get all distance values for debugging ---
//                double finalDistance = turret.getTargetDistance(drivetrain);
//                double odoDistance = turret.getTargetDistanceByOdometry(drivetrain);
//                double angleDistance = turret.getTargetDistanceByAngle();
//
//                // --- Send all relevant distances as key-value pairs ---
//                client.sendKeyValue("dist_final_in", String.format(Locale.US, "%.2f", finalDistance));
//                client.sendKeyValue("dist_odo_in", String.format(Locale.US, "%.2f", odoDistance));
//                client.sendKeyValue("dist_angle_in", String.format(Locale.US, "%.2f", angleDistance));
//
//                // --- Draw the line based on the final, chosen distance ---
//                if (finalDistance > 0) {
//                    // Calculate the position of the front of the robot
//                    double lineStartX = robotX + ROBOT_FRONT_OFFSET_INCHES * Math.cos(robotHeadingRad);
//                    double lineStartY = robotY + ROBOT_FRONT_OFFSET_INCHES * Math.sin(robotHeadingRad);
//
//                    // Calculate the direction vector from the *front of the robot* to the tag
//                    double vecX = tagX - lineStartX;
//                    double vecY = tagY - lineStartY;
//                    double mag = Math.hypot(vecX, vecY);
//                    double dirX = vecX / mag;
//                    double dirY = vecY / mag;
//
//                    // Calculate endpoint for the line using the final chosen distance
//                    double lineEndX = lineStartX + dirX * finalDistance;
//                    double lineEndY = lineStartY + dirY * finalDistance;
//
//                    int lineStyle;
//                    if (mode == Turret.TurretMode.ODOMETRY_AIM) {
//                        lineStyle = 2;
//                    } else {
//                        lineStyle = 1;
//                    }
//
//                    // Send the line starting from the calculated front position
//                    client.sendLine("dist_line", lineStartX, lineStartY, lineEndX, lineEndY, lineStyle);
//                } else {
//                    // If the final distance is zero, don't draw a line
//                    client.sendLine("dist_line", 0, 0, 0, 0, 0);
//                }
//            }
//        } else {
//            // If not in an aiming mode, clear the lines and values
//            client.sendLine("dist_line", 0, 0, 0, 0, 0);
//            client.sendKeyValue("dist_final_in", "0.00");
//            client.sendKeyValue("dist_odo_in", "0.00");
//            client.sendKeyValue("dist_angle_in", "0.00");
//        }

//        // --- Send AprilTag Line of Sight ---
//        if (turret.getMode() == Turret.TurretMode.AUTO_AIM && turret.isTargetVisible()) {
//            double tagX = 0;
//            double tagY = 0;
//            boolean targetFound = false;
//
//            Alliance.Color target = turret.getTargetAlliance();
//            if (target == Alliance.Color.BLUE) {
//                tagX = BLUE_APRILTAG_X;
//                tagY = BLUE_APRILTAG_Y;
//                targetFound = true;
//            } else if (target == Alliance.Color.RED) {
//                tagX = RED_APRILTAG_X;
//                tagY = RED_APRILTAG_Y;
//                targetFound = true;
//            }
//
//            if (targetFound) {
//                // Get both distance calculations
//                double distFromArea = turret.getTargetDistanceByArea();
//                double distFromAngle = turret.getTargetDistanceByAngle();
//
//                // Send both distances as key-value pairs for comparison
//                client.sendKeyValue("dist_area_in", String.format(Locale.US, "%.2f", distFromArea));
//                client.sendKeyValue("dist_angle_in", String.format(Locale.US, "%.2f", distFromAngle));
//
//                // Calculate the position of the front of the robot
//                double lineStartX = robotX + ROBOT_FRONT_OFFSET_INCHES * Math.cos(robotHeadingRad);
//                double lineStartY = robotY + ROBOT_FRONT_OFFSET_INCHES * Math.sin(robotHeadingRad);
//
//                // Calculate the direction vector from the *front of the robot* to the tag
//                double vecX = tagX - lineStartX;
//                double vecY = tagY - lineStartY;
//                double mag = Math.hypot(vecX, vecY);
//                double dirX = vecX / mag;
//                double dirY = vecY / mag;
//
//                // Calculate endpoint for the angle-based distance line, starting from the robot's front
//                double lineEndAngleX = lineStartX + dirX * distFromAngle;
//                double lineEndAngleY = lineStartY + dirY * distFromAngle;
//
//                // Calculate endpoint for the area-based distance line, starting from the robot's front
//                // (Even though it's commented out, we'll update its logic too for completeness)
//                double lineEndAreaX = lineStartX + dirX * distFromArea;
//                double lineEndAreaY = lineStartY + dirY * distFromArea;
//
//                // Send the line starting from the calculated front position
//                client.sendLine("dist_line_angle", lineStartX, lineStartY, lineEndAngleX, lineEndAngleY, 1);
//                // client.sendLine("dist_line_area", lineStartX, lineStartY, lineEndAreaX, lineEndAreaY, 2);
//
//            }
//        } else {
//            // If not aiming or target not visible, clear the lines and values
//            client.sendLine("dist_line_angle", 0, 0, 0, 0, 0);
//            client.sendLine("dist_line_area", 0, 0, 0, 0, 0);
//            client.sendKeyValue("dist_area_in", "0.00");
//            client.sendKeyValue("dist_angle_in", "0.00");
//        }

//        // Send back wheel gain only if it has changed
//        double currentBackWheelGain = drivetrain.getBackWheelGain();
//        if (currentBackWheelGain != prevBackWheelGain) {
//            client.sendKeyValue("backWheelGain", String.format(Locale.US, "%.2f", currentBackWheelGain));
//            prevBackWheelGain = currentBackWheelGain;
//        }

        // Get ball colors from the Intake subsystem and send if they have changed
//        BallColor.DetectedColor currentLeftColor = intake.getBallColor(0);
//        if (currentLeftColor != prevLeftColor) {
//            client.sendKeyValue("leftColor", currentLeftColor.toString());
//            prevLeftColor = currentLeftColor;
//        }
//
//        BallColor.DetectedColor currentMidColor = intake.getBallColor(1);
//        if (currentMidColor != prevMidColor) {
//            client.sendKeyValue("midColor", currentMidColor.toString());
//            prevMidColor = currentMidColor;
//        }
//
//        BallColor.DetectedColor currentRightColor = intake.getBallColor(2);
//        if (currentRightColor != prevRightColor) {
//            client.sendKeyValue("rightColor", currentRightColor.toString());
//            prevRightColor = currentRightColor;
        }
//    }

//    /**
//     * The main update loop for the simulator. Gathers data from other subsystems
//     * and sends it over UDP if the simulator is active.
//     * @param drivetrain The drivetrain subsystem to get pose and gain data from.
//     * @param intake The intake subsystem to get ball color data from.
//     */
//    public void update(Drivetrain drivetrain, Intake intake) {
//        if (!isActive || client == null || !client.isInitialized()) {
//            return;
//        }
//
//        // Send robot pose
//        Pose2D currentPose = drivetrain.getPose();
//        client.sendPosition(
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.DEGREES)
//        );
//
//        // Send back wheel gain only if it has changed
//        double currentBackWheelGain = drivetrain.getBackWheelGain();
//        if (currentBackWheelGain != prevBackWheelGain) {
//            client.sendKeyValue("backWheelGain", String.format(Locale.US, "%.2f", currentBackWheelGain));
//            prevBackWheelGain = currentBackWheelGain;
//        }
//
//        // Get ball colors from the Intake subsystem and send if they have changed
//        BallColor.DetectedColor currentLeftColor = intake.getBallColor(0);
//        if (currentLeftColor != prevLeftColor) {
//            client.sendKeyValue("leftColor", currentLeftColor.toString());
//            prevLeftColor = currentLeftColor;
//        }
//
//        BallColor.DetectedColor currentMidColor = intake.getBallColor(1);
//        if (currentMidColor != prevMidColor) {
//            client.sendKeyValue("midColor", currentMidColor.toString());
//            prevMidColor = currentMidColor;
//        }
//
//        BallColor.DetectedColor currentRightColor = intake.getBallColor(2);
//        if (currentRightColor != prevRightColor) {
//            client.sendKeyValue("rightColor", currentRightColor.toString());
//            prevRightColor = currentRightColor;
//        }
//    }

    /**
     * Closes the UDP client connection.
     */
    public void close() {
        if (client != null) {
            client.close();
        }
        client = null;
        isActive = false;
    }
}
