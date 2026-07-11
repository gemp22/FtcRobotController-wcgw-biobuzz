package org.firstinspires.ftc.teamcode.pathing;

/**
 * A simple data class to represent a 2D point with double-precision coordinates.
 * This avoids conflicts with Android's integer-based Point class.
 */
public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {
        this.x = 0;
        this.y = 0;
    }
}
