package org.firstinspires.ftc.teamcode.pathing;

import static org.firstinspires.ftc.teamcode.pathing.MathUtil.AngleWrap;
import static org.firstinspires.ftc.teamcode.pathing.MathUtil.clipToLine;
import static org.firstinspires.ftc.teamcode.pathing.MathUtil.lineCircleIntersection;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.UdpClientFieldSim;
import java.util.ArrayList;
import java.util.Locale;

/**
 * A utility class that contains the logic for a pure pursuit-style path following algorithm.
 * It takes the robot's current state and a path, and returns the calculated motor powers.
 */
public class PathFollower {
    // --- Gain for slip prediction tuning ---
    public static double SLIP_GAIN = 1.0;

    private boolean isDebugEnabled = false;
    private UdpClientFieldSim clientSim;
    private String clientSimHost = "192.168.43.100"; // Default localhost
    private int clientSimPort = 7777;         // Default port
    private ArrayList<CurvePoint> lastSentPath = null;

    // --- State variables for path following ---
    private boolean isOnLastSegment = false;
    private double lastFollowPointDistToEnd = Double.MAX_VALUE;
    private boolean hasOvershotFinalPoint = false;

    // --- Timer for dynamic prediction ---
    private ElapsedTime predictionTimer;

    /**
     * A simple data class to hold the calculated motor powers and the status of the path.
     */
    public static class PathingResult {
        public double moveX;
        public double moveY;
        public double turn;
        public boolean isFinished;

        public PathingResult(double moveX, double moveY, double turn, boolean isFinished) {
            this.moveX = moveX;
            this.moveY = moveY;
            this.turn = turn;
            this.isFinished = isFinished;
        }
    }

    /**
     * Constructs a new PathFollower and initializes its state, including the prediction timer.
     */
    public PathFollower() {
        predictionTimer = new ElapsedTime();
        reset();
    }

    /**
     * Enables or disables debugging visuals sent over UDP to the field simulator.
     * This will create the UDP client on the first time it's enabled.
     * @param enable True to enable, false to disable.
     */
    public void setDebug(boolean enable) {
        if (enable && !this.isDebugEnabled) {
            // Turning debugging ON
            this.isDebugEnabled = true;
            if (clientSim == null) {
                try {
                    clientSim = new UdpClientFieldSim(clientSimHost, clientSimPort);
                } catch (Exception e) {
                    clientSim = null; // Stay null if creation fails
                }
            }
        } else if (!enable && this.isDebugEnabled) {
            // Turning debugging OFF
            this.isDebugEnabled = false;
            // Clear any lingering lines from the simulator
            if (clientSim != null && lastSentPath != null) {
                for (int i = 0; i < lastSentPath.size(); i++) {
                    clientSim.sendLine("seg_" + i, 0,0,0,0, 0);
                }
            }
            lastSentPath = null;
            close(); // Close the client when disabling
        }
    }

    /**
     * The main path-following method. It calculates the necessary robot-centric motor powers
     * to follow a given path from its current position, compensating for predicted slip.
     *
     * @param path              The list of CurvePoints defining the path for the robot to follow.
     * @param robotPose         The robot's current position (x, y) and heading, obtained from odometry.
     * @param robotLinVel       The robot's current translational (linear) velocity (x, y) from odometry.
     * @param robotAngVel       The robot's current angular velocity (turn speed) in radians per second.
     * @param followAngle       The desired angle for the robot to maintain relative to its direction of travel along the path.
     *                          For example, 90 degrees means the robot's "front" faces along the path.
     * @param tolerance         The distance (in inches) from the final point of the path at which the path is considered completed.
     * @return                  A PathingResult object containing the calculated robot-centric x, y, and turn powers,
     *                          and a boolean indicating if the path is finished.
     */
    public PathingResult followCurve(ArrayList<CurvePoint> path, Pose2D robotPose, Velocity robotLinVel, double robotAngVel, double followAngle, double tolerance, boolean debug) {
        if (path == null || path.size() < 2) {
            return new PathingResult(0, 0, 0, true);
        }

        // --- 1. FIND TARGET POINTS ---
        PointWithIndex clippedToPath = clipToPath(path, robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH));
        int currFollowIndex = clippedToPath.index + 1;
        CurvePoint currentPoint = path.get(currFollowIndex);

        CurvePoint followMe = getFollowPointPath(path, robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH), currentPoint.followDistance, debug);

        double distToFinalEnd = Math.hypot(robotPose.getX(DistanceUnit.INCH) - path.get(path.size() - 1).x, robotPose.getY(DistanceUnit.INCH) - path.get(path.size() - 1).y);

        // --- 2. CALCULATE POWERS ---
        PathingResult result = moveToPoint(followMe, robotPose, robotLinVel, robotAngVel, followAngle, debug, currentPoint.followDistance);

        // --- 3. SET FINAL STATE ---
//        result.isFinished = (distToFinalEnd < tolerance)||(this.hasOvershotFinalPoint);
//        result.isFinished = (distToFinalEnd < tolerance);
        // Add overshoot detection logic
        boolean isGettingFarther = false;
        // Only check for overshoot if we are close to the end point to avoid false positives early in the path.
        if (distToFinalEnd < 5.0) {
            if (distToFinalEnd > this.lastFollowPointDistToEnd) {
                isGettingFarther = true;
            }
        }
        // The path is finished if we are within the tolerance OR if we have started moving away from the final point.
        result.isFinished = (distToFinalEnd < tolerance) || isGettingFarther;

        // Update the last known distance for the next loop's comparison.
        this.lastFollowPointDistToEnd = distToFinalEnd;


        // --- DEBUG (Path-level) ---
        if (isDebugEnabled && debug && clientSim != null) {
            sendFollowCurveDebugData(path, robotPose, clippedToPath, followMe, currFollowIndex);
        }

        return result;
    }

    /**
     * Calculates the motor powers required to move towards a target point and aim at another,
     * including slip and turn compensation. This is the "how to get there" part of the algorithm.
     * @param followMe The target point to drive towards.
     * @param robotPose The robot's current position and heading.
     * @param robotLinVel The robot's current linear velocity.
     * @param robotAngVel The robot's current angular velocity.
     * @param followAngle The desired angle of the robot relative to the path.
     * @param debug Whether to send debug data for this specific calculation.
     * @param followDistance The current follow distance, passed for debug visualization.
     * @return A PathingResult containing the calculated moveX, moveY, and turn powers.
     */
    private PathingResult moveToPoint(CurvePoint followMe, Pose2D robotPose, Velocity robotLinVel, double robotAngVel, double followAngle, boolean debug, double followDistance) {
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotAngleRad = robotPose.getHeading(AngleUnit.RADIANS);

        // --- DYNAMIC PREDICTION TIME ---
        double predictionTime = predictionTimer.seconds();
        // Review me, as it works better with this hardcoded predictionTime tahn with timer
        predictionTime = 0.1;

        // Reset the timer for the next cycle
        predictionTimer.reset();
        // ---

        // --- 1. SLIP PREDICTION ---
        Velocity velInches = robotLinVel.toUnit(DistanceUnit.INCH);
        double worldSlipX = velInches.xVeloc * predictionTime * SLIP_GAIN;
        double worldSlipY = velInches.yVeloc * predictionTime * SLIP_GAIN;
        double targetXAdjusted = followMe.x - worldSlipX;
        double targetYAdjusted = followMe.y - worldSlipY;

        // --- 2. MOVEMENT CALCULATION ---
        double angleToAdjustedPoint = Math.atan2(targetYAdjusted - robotY, targetXAdjusted - robotX);
        double distanceToAdjustedPoint = Math.hypot(targetXAdjusted - robotX, targetYAdjusted - robotY);

        double robotCentricAngleToTarget = AngleWrap(angleToAdjustedPoint - robotAngleRad);

        // Project the robot-centric vector onto the robot's axes.
        // Your Drivetrain.drive(y, x, rx) expects y=forward, x=strafe RIGHT.
        // Standard trig (cos for X, sin for Y) assumes X=forward, Y=LEFT.
        // Therefore, we use cos for forward and -sin for strafe RIGHT.
        double forwardPower = Math.cos(robotCentricAngleToTarget);
        double strafePower  = -Math.sin(robotCentricAngleToTarget);

        double relativeXToPoint = strafePower * distanceToAdjustedPoint;
        double relativeYToPoint = forwardPower * distanceToAdjustedPoint;

        double powerDenominator = Math.max(1.0, Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double moveX = (relativeXToPoint / powerDenominator) * followMe.moveSpeed;
        double moveY = (relativeYToPoint / powerDenominator) * followMe.moveSpeed;

        // --- 3. TURN CALCULATION ---
        double angleToPointTo = Math.atan2(followMe.y - robotY, followMe.x - robotX);
        double targetRobotHeading = angleToPointTo + (followAngle - Math.toRadians(90));
        double angularSlipRad = robotAngVel * predictionTime * SLIP_GAIN;
        double turnError = AngleWrap(targetRobotHeading - robotAngleRad - angularSlipRad);
        double turn = Range.clip(turnError / Math.toRadians(45.0), -followMe.turnSpeed, followMe.turnSpeed);

        // --- 4. FINAL ADJUSTMENTS ---
        double turnErrorReduction = 1.0 - Range.clip(Math.abs(turnError) / followMe.slowDownTurnRadians, 0, followMe.slowDownTurnAmount);
        moveX *= turnErrorReduction;
        moveY *= turnErrorReduction;

        // --- DEBUG (moveToPoint level) ---
        if (isDebugEnabled && debug && clientSim != null) {
            sendMoveToPointDebugData(followMe, targetXAdjusted, targetYAdjusted, targetRobotHeading, followDistance, predictionTime, turnError, turn, turnErrorReduction);
        }

        return new PathingResult(moveX, moveY, turn, false);
    }

    /**
     * Sends debug data specific to the overall path-following logic.
     * This includes the full path, robot position, and target points.
     * @param path The full path being followed.
     * @param robotPose The robot's current pose.
     * @param clippedToPath The point on the path closest to the robot.
     * @param followMe The calculated "carrot" point to drive towards.
     * @param currFollowIndex The index of the current path segment being targeted.
     */
    private void sendFollowCurveDebugData(ArrayList<CurvePoint> path, Pose2D robotPose, PointWithIndex clippedToPath, CurvePoint followMe, int currFollowIndex) {
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);

        clientSim.sendPosition(robotX, robotY, robotPose.getHeading(AngleUnit.DEGREES));

        if (!path.equals(lastSentPath)) {
            for (int i = 0; i < path.size() - 1; i++) {
                CurvePoint p1 = path.get(i);
                CurvePoint p2 = path.get(i + 1);
                clientSim.sendLine("seg_" + i, p1.x, p1.y, p2.x, p2.y, 2);
            }
            lastSentPath = (ArrayList<CurvePoint>) path.clone();
        }

        clientSim.sendLine("robot_to_clipped", robotX, robotY, clippedToPath.x, clippedToPath.y, 2);
        clientSim.sendLine("robot_to_followMe", robotX, robotY, followMe.x, followMe.y, 1);

        double markerSize = 1.5;
        clientSim.sendLine("followMe_H", followMe.x - markerSize, followMe.y, followMe.x + markerSize, followMe.y, 1);
        clientSim.sendLine("followMe_V", followMe.x, followMe.y - markerSize, followMe.x, followMe.y + markerSize, 1);

        clientSim.sendKeyValue("pf.isOnLastSegment", String.valueOf(this.isOnLastSegment));

        clientSim.sendText(String.format(Locale.US, "Following Path Segment: %d", currFollowIndex - 1));
    }

    /**
     * Sends debug data specific to the internal calculations of moveToPoint.
     * This includes slip compensation values and target headings.
     * @param followMe The raw target point before adjustments.
     * @param targetXAdjusted The slip-compensated X coordinate.
     * @param targetYAdjusted The slip-compensated Y coordinate.
     * @param targetRobotHeading The final calculated world heading for the robot.
     * @param followDistance The radius of the follow-circle being drawn.
     * @param predictionTime The last measured loop time used for slip prediction.
     * @param turnError The calculated turn error in radians.
     * @param turn The final calculated turn power.
     * @param turnErrorReduction The calculated movement reduction factor.
     */
    private void sendMoveToPointDebugData(CurvePoint followMe, double targetXAdjusted, double targetYAdjusted, double targetRobotHeading, double followDistance, double predictionTime, double turnError, double turn, double turnErrorReduction) {
        // Send the follow-distance circle around the robot with an indicator for the final calculated target heading.
        clientSim.sendCircle(followDistance, Math.toDegrees(targetRobotHeading));

        // Send key-value pairs for detailed inspection
        clientSim.sendKeyValue("pf.followMe.x", String.format(Locale.US, "%.1f", followMe.x));
        clientSim.sendKeyValue("pf.followMe.y", String.format(Locale.US, "%.1f", followMe.y));
        clientSim.sendKeyValue("pf.target_adj.x", String.format(Locale.US, "%.1f", targetXAdjusted));
        clientSim.sendKeyValue("pf.target_adj.y", String.format(Locale.US, "%.1f", targetYAdjusted));
        clientSim.sendKeyValue("pf.target_heading_deg", String.format(Locale.US, "%.1f", Math.toDegrees(targetRobotHeading)));
        clientSim.sendKeyValue("pf.prediction_time_ms", String.format(Locale.US, "%.1f", predictionTime * 1000));

        clientSim.sendKeyValue("pf.turnError_deg", String.format(Locale.US, "%.1f", Math.toDegrees(turnError)));
        clientSim.sendKeyValue("pf.turn_power", String.format(Locale.US, "%.2f", turn));
        clientSim.sendKeyValue("pf.turnErrorReduction", String.format(Locale.US, "%.2f", turnErrorReduction));
    }

    /**
     * Resets any internal state of the path follower, such as debug data and timers.
     * This should be called when starting a new, unrelated path sequence.
     */
    public void reset() {
        this.lastSentPath = null;
        this.isOnLastSegment = false;
        this.lastFollowPointDistToEnd = Double.MAX_VALUE;
        this.hasOvershotFinalPoint = false;
        if (this.predictionTimer != null) {
            this.predictionTimer.reset();
        }
    }

    /**
     * Closes any open resources, like the UDP client used for debugging.
     * This should be called when the OpMode is stopped to release the socket.
     */
    public void close() {
        if (clientSim != null) {
            clientSim.close();
            clientSim = null;
        }
        this.isDebugEnabled = false;
    }

    /**
     * A helper data class to associate a point with a path index.
     */
    private class PointWithIndex {
        private final double x;
        private final double y;
        private final int index;

        public PointWithIndex(double xPos, double yPos, int index) {
            this.x = xPos;
            this.y = yPos;
            this.index = index;
        }
    }

    /**
     * Finds the closest point on the entire path to the robot.
     * @return A PointWithIndex containing the coordinates of the closest point and the index of the segment it's on.
     */
    private PointWithIndex clipToPath(ArrayList<CurvePoint> path, double robotX, double robotY) {
        double closestDistance = Double.MAX_VALUE;
        int closestSegmentIndex = 0;
        Point closestPoint = new Point();

        for (int i = 0; i < path.size() - 1; i++) {
            CurvePoint p1 = path.get(i);
            CurvePoint p2 = path.get(i + 1);

            Point clipped = clipToLine(p1.x, p1.y, p2.x, p2.y, robotX, robotY);
            double distance = Math.hypot(robotX - clipped.x, robotY - clipped.y);

            if (distance < closestDistance) {
                closestDistance = distance;
                closestSegmentIndex = i;
                closestPoint = clipped;
            }
        }
        return new PointWithIndex(closestPoint.x, closestPoint.y, closestSegmentIndex);
    }

    /**
     * Finds the index of the path segment the robot should currently be targeting by finding the
     * closest vertex (start of a segment) to the robot's current position.
     *
     * @param path   The list of CurvePoints defining the path.
     * @param robotX The robot's current X coordinate.
     * @param robotY The robot's current Y coordinate.
     * @return The index of the point that starts the closest segment. For example, if the robot is
     *         closest to path.get(2), this returns 3 (representing the segment from index 2 to 3).
     *         If the robot is past the second-to-last point, it returns the index of the last point.
     */
    private int findClosestPathSegment(ArrayList<CurvePoint> path, double robotX, double robotY) {
        if (path == null || path.isEmpty()) {
            return 0; // No path, no segment.
        }
        if (path.size() == 1) {
            return 0; // Only one point, it's segment 0.
        }

        int closestIndex = 0;
        double minDistanceSq = Double.MAX_VALUE;

        // Iterate through all points in the path to find the one closest to the robot.
        // We only need to check up to the second-to-last point, as the last point doesn't start a segment.
        for (int i = 0; i < path.size() -1; i++) {
            CurvePoint p = path.get(i);
            double dx = robotX - p.x;
            double dy = robotY - p.y;
            double distanceSq = dx * dx + dy * dy; // Use squared distance to avoid sqrt

            if (distanceSq < minDistanceSq) {
                minDistanceSq = distanceSq;
                closestIndex = i;
            }
        }

        // The segment to follow starts at the closest vertex and goes to the next one.
        // We return the index of the *end* point of that segment.
        int targetPointIndex = closestIndex + 1;

        // Ensure the index does not go past the last point.
        // This is already handled by the loop condition, but it's good practice.
        // If the closest vertex found was the second-to-last one, targetPointIndex will be the last index.
        return targetPointIndex;
    }

    /**
     * Finds the "carrot on a stick" point to follow along the path.
     * @return The CurvePoint that the robot should drive towards.
     */
    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> path, double robotX, double robotY, double followRadius, boolean debug) {
        CurvePoint finalPoint = path.get(path.size() - 1);

        // --- 0. CHECK IF LOCKED ON FINAL POINT ---
        // If we have previously overshot the end, lock onto the final point forever.
        if (this.hasOvershotFinalPoint) {
            return finalPoint;
        }

        // --- 1. COLLECT ALL INTERSECTIONS ---
        ArrayList<PointWithIndex> allIntersections = new ArrayList<>();
        for (int i = 0; i < path.size() - 1; i++) {
            CurvePoint startLine = path.get(i);
            CurvePoint endLine = path.get(i + 1);

            ArrayList<Point> intersectionsOnSegment = lineCircleIntersection(robotX, robotY, followRadius, startLine.x, startLine.y, endLine.x, endLine.y);
            for (Point intersection : intersectionsOnSegment) {
                allIntersections.add(new PointWithIndex(intersection.x, intersection.y, i));
            }
        }

        // --- 2. HANDLE NO INTERSECTIONS ---
        if (allIntersections.isEmpty()) {
            // If the robot (circle with follow radious) is not touching the path, find the closest point on the path and use that.
            int closestIndex = findClosestPathSegment(path, robotX, robotY);
            CurvePoint followMe = path.get(closestIndex).clone();
            return followMe;
        }

        // --- 3. SELECT THE BEST INTERSECTION ---
        PointWithIndex bestIntersection = allIntersections.get(0);

        for (int i = 1; i < allIntersections.size(); i++) {
            PointWithIndex currentIntersection = allIntersections.get(i);

            if (currentIntersection.index > bestIntersection.index) {
                // This intersection is on a more advanced segment, so it's automatically better.
                bestIntersection = currentIntersection;
            } else if (currentIntersection.index == bestIntersection.index) {
                // Tie-breaker: both intersections are on the same segment.
                // Pick the one that is closer to the END of that segment.
                CurvePoint segmentEnd = path.get(currentIntersection.index + 1);

                double distCurrentToEnd = Math.hypot(currentIntersection.x - segmentEnd.x, currentIntersection.y - segmentEnd.y);
                double distBestToEnd = Math.hypot(bestIntersection.x - segmentEnd.x, bestIntersection.y - segmentEnd.y);

                if (distCurrentToEnd < distBestToEnd) {
                    // The current intersection is farther along the same segment.
                    bestIntersection = currentIntersection;
                }
            }
        }

        // --- 4. CREATE THE FOLLOW POINT ---
        // Create a new CurvePoint based on the best intersection found.
        // It inherits properties from the END point of the segment it's on.
        CurvePoint followMe = path.get(bestIntersection.index + 1).clone();
        followMe.setPoint(new Point(bestIntersection.x, bestIntersection.y));

        // --- 5. OVERSHOOT DETECTION LOGIC ---
        // Update the internal flag based on the segment of the chosen intersection point.
        if (!this.isOnLastSegment) {
            this.isOnLastSegment = (bestIntersection.index == path.size() - 2);
        }

        if (this.isOnLastSegment) {
            double followPointDistToEnd = Math.hypot(followMe.x - finalPoint.x, followMe.y - finalPoint.y);

            if (isDebugEnabled && debug && clientSim != null) {
                clientSim.sendKeyValue("fpp.followMe.x", String.format(Locale.US, "%.1f", followMe.x));
                clientSim.sendKeyValue("fpp.followMe.y", String.format(Locale.US, "%.1f", followMe.y));
                clientSim.sendKeyValue("fpp.finalPoint.x", String.format(Locale.US, "%.1f", finalPoint.x));
                clientSim.sendKeyValue("fpp.finalPoint.y", String.format(Locale.US, "%.1f", finalPoint.y));
                clientSim.sendKeyValue("fpp.hasOvershotFinalPoint", String.valueOf(this.hasOvershotFinalPoint));

                clientSim.sendKeyValue("fpp.followPointDistToEnd", String.format(Locale.US, "%.1f", followPointDistToEnd));
                clientSim.sendKeyValue("fpp.lastFollowPointDistToEnd", String.format(Locale.US, "%.1f", this.lastFollowPointDistToEnd));
            }

            // If the calculated follow point is now farther from the end than the last one was,
            // it means we have overshot the end or the intersection is the one on the back of the robot
            if (followPointDistToEnd > this.lastFollowPointDistToEnd) {
                this.hasOvershotFinalPoint = true;
                return finalPoint; // Lock to the final point.
            }

            // Otherwise, update the last known "good" distance.
            this.lastFollowPointDistToEnd = followPointDistToEnd;
        }

        // --- 6. RETURN THE RESULT ---
        return followMe;
    }

    /**
     * Creates a new CurvePoint that extends a line segment by a given distance.
     * @return A new CurvePoint representing the extended endpoint.
     */
    private CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {
        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y, secondPoint.x - firstPoint.x);
        double extendedX = secondPoint.x + distance * Math.cos(lineAngle);
        double extendedY = secondPoint.y + distance * Math.sin(lineAngle);

        CurvePoint extended = secondPoint.clone();
        extended.x = extendedX;
        extended.y = extendedY;
        return extended;
    }
}
