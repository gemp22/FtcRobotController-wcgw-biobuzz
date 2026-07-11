package org.firstinspires.ftc.teamcode.pathing;

import java.util.ArrayList;

/**
 * A utility class containing static mathematical functions needed for pure pursuit
 * and other path-following algorithms.
 */
public final class MathUtil {

    /**
     * Private constructor to prevent instantiation of this utility class.
     */
    private MathUtil() {}

    /**
     * Wraps an angle in radians to the range -PI to +PI.
     * @param angle The angle in radians.
     * @return The wrapped angle.
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    /**
     * Finds the intersection points between a line segment and a circle.
     * @param cx Circle center X.
     * @param cy Circle center Y.
     * @param radius Circle radius.
     * @param x1 Line segment start X.
     * @param y1 Line segment start Y.
     * @param x2 Line segment end X.
     * @param y2 Line segment end Y.
     * @return A list of intersection points (can be 0, 1, or 2 points).
     */
    public static ArrayList<Point> lineCircleIntersection(double cx, double cy, double radius,
                                                          double x1, double y1, double x2, double y2) {
        ArrayList<Point> intersections = new ArrayList<>();

        double dx = x2 - x1;
        double dy = y2 - y1;

        double a = dx * dx + dy * dy;
        double b = 2 * (dx * (x1 - cx) + dy * (y1 - cy));
        double c = (x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy) - radius * radius;

        double det = b * b - 4 * a * c;
        if (a <= 0.0000001 || det < 0) {
            // No real solutions
            return intersections;
        } else {
            double t1 = (-b + Math.sqrt(det)) / (2 * a);
            double t2 = (-b - Math.sqrt(det)) / (2 * a);

            // Check if the intersection points are within the line segment (0 <= t <= 1)
            if (t1 >= 0 && t1 <= 1) {
                intersections.add(new Point(x1 + t1 * dx, y1 + t1 * dy));
            }
            if (t2 >= 0 && t2 <= 1) {
                intersections.add(new Point(x1 + t2 * dx, y1 + t2 * dy));
            }
            return intersections;
        }
    }

    /**
     * Clips a robot's position to the closest point on an infinite line defined by two points.
     * @param lineX1 First point X on the line.
     * @param lineY1 First point Y on the line.
     * @param lineX2 Second point X on the line.
     * @param lineY2 Second point Y on the line.
     * @param robotX The robot's X position.
     * @param robotY The robot's Y position.
     * @return The new Point representing the closest location on the line.
     */
    public static Point clipToLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                   double robotX, double robotY) {
        // Avoid division by zero
        if (Math.abs(lineX1 - lineX2) < 0.01) { lineX1 += 0.01; }
        if (Math.abs(lineY1 - lineY2) < 0.01) { lineY1 += 0.01; }


        // Calculate the slope of the line
        double m1 = (lineY2 - lineY1) / (lineX2 - lineX1);
        // Calculate the slope of the perpendicular line
        double m2 = (lineX1 - lineX2) / (lineY2 - lineY1);

        // Find the intersection of the two lines
        double clippedX = ((-m2 * robotX) + robotY + (m1 * lineX1) - lineY1) / (m1 - m2);
        double clippedY = (m1 * (clippedX - lineX1)) + lineY1;

        return new Point(clippedX, clippedY);
    }
}
