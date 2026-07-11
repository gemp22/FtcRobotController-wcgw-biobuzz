package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import java.util.ArrayList;

/**
 * A data class to hold the results from the ListenForCurvePoints singleton.
 */
public class ListenResult {
    private final ListenStatus status;
    private final ArrayList<CurvePoint> points;
    private final String errorMessage;
    private final double startPoseX;
    private final double startPoseY;
    private final double startPoseHeading;
    private final double followAngle; // NEW

    public ListenResult(ListenStatus status, ArrayList<CurvePoint> points, String errorMessage, double startPoseX, double startPoseY, double startPoseHeading, double followAngle) {
        this.status = status;
        this.points = points;
        this.errorMessage = errorMessage;
        this.startPoseX = startPoseX;
        this.startPoseY = startPoseY;
        this.startPoseHeading = startPoseHeading;
        this.followAngle = followAngle; // NEW
    }

    public ListenStatus getStatus() {
        return status;
    }

    public ArrayList<CurvePoint> getPoints() {
        return points;
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public double getStartPoseX() {
        return startPoseX;
    }

    public double getStartPoseY() {
        return startPoseY;
    }

    public double getStartPoseHeading() {
        return startPoseHeading;
    }

    // --- NEW: Getter for the follow angle ---
    public double getFollowAngle() {
        return followAngle;
    }

    /**
     * Helper method to check if a valid start pose was received.
     * @return true if the start pose was received, false otherwise.
     */
    public boolean hasStartPose() {
        return !Double.isNaN(startPoseX);
    }
}


//package org.firstinspires.ftc.teamcode.util;
//
//import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
//import java.util.ArrayList;
//
///**
// * A data class to hold the results from the ListenForCurvePoints singleton.
// */
//public class ListenResult {
//    private final ListenStatus status;
//    private final ArrayList<CurvePoint> points;
//    private final String errorMessage;
//    // --- NEW: Fields for the start pose ---
//    private final double startPoseX;
//    private final double startPoseY;
//    private final double startPoseHeading;
//
//    public ListenResult(ListenStatus status, ArrayList<CurvePoint> points, String errorMessage, double startPoseX, double startPoseY, double startPoseHeading) {
//        this.status = status;
//        this.points = points;
//        this.errorMessage = errorMessage;
//        // --- NEW: Assign start pose ---
//        this.startPoseX = startPoseX;
//        this.startPoseY = startPoseY;
//        this.startPoseHeading = startPoseHeading;
//    }
//
//    public ListenStatus getStatus() {
//        return status;
//    }
//
//    public ArrayList<CurvePoint> getPoints() {
//        return points;
//    }
//
//    public String getErrorMessage() {
//        return errorMessage;
//    }
//
//    // --- NEW: Getters for the start pose ---
//    public double getStartPoseX() {
//        return startPoseX;
//    }
//
//    public double getStartPoseY() {
//        return startPoseY;
//    }
//
//    public double getStartPoseHeading() {
//        return startPoseHeading;
//    }
//
//    /**
//     * Helper method to check if a valid start pose was received.
//     * @return true if the start pose was received, false otherwise.
//     */
//    public boolean hasStartPose() {
//        return !Double.isNaN(startPoseX);
//    }
//}
