package org.firstinspires.ftc.teamcode.util;

/**
 * This class defines the keys used for storing and retrieving data from the OpMode blackboard.
 * Using a central class for keys prevents typos and makes the code easier to manage.
 */
public class Blackboard {
    /**
     * The key used to store the alliance color (as a Alliance.Color enum) from Auto to TeleOp.
     */
    public static final String ALLIANCE_KEY = "AllianceColor";
    public static final String ROBOT_POSE_KEY = "robot_pose";
    public static final String TURRET_ANGLE_KEY = "turret_angle";
}
