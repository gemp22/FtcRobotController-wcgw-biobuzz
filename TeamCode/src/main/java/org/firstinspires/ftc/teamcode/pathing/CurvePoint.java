package org.firstinspires.ftc.teamcode.pathing; // <<< CHANGED

import java.util.Objects;

// Note: This class depends on a `PointDouble` class. I will create a standard
// version of it in the next step, named `Point`.
public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed,
                      double followDistance, double slowDownTurnRadians, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint nextPoint) {
        x = nextPoint.x;
        y = nextPoint.y;
        moveSpeed = nextPoint.moveSpeed;
        turnSpeed = nextPoint.turnSpeed;
        followDistance = nextPoint.followDistance;
        slowDownTurnRadians = nextPoint.slowDownTurnRadians;
        slowDownTurnAmount = nextPoint.slowDownTurnAmount;
    }

    public Point toPoint(){
        return new Point(x,y);
    }

    public void setPoint(Point p){
        x = p.x;
        y = p.y;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        CurvePoint that = (CurvePoint) o;
        return Double.compare(that.x, x) == 0 &&
                Double.compare(that.y, y) == 0 &&
                Double.compare(that.moveSpeed, moveSpeed) == 0 &&
                Double.compare(that.turnSpeed, turnSpeed) == 0 &&
                Double.compare(that.followDistance, followDistance) == 0 &&
                Double.compare(that.slowDownTurnRadians, slowDownTurnRadians) == 0 &&
                Double.compare(that.slowDownTurnAmount, slowDownTurnAmount) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, moveSpeed, turnSpeed, followDistance, slowDownTurnRadians, slowDownTurnAmount);
    }

    // It's useful to add a clone method for copying paths
    @Override
    public CurvePoint clone() {
        return new CurvePoint(this);
    }
}
