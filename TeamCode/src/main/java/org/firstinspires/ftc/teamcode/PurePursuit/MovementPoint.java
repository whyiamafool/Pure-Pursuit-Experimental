package org.firstinspires.ftc.teamcode.PurePursuit;

public class MovementPoint {

    private double x;
    private double y;
    private double theta; //between 0 and 2pi
    private double tolerance;

    public MovementPoint(double x, double y, double theta, double tolerance) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.tolerance = tolerance;
    }

    public MovementPoint(double x, double y, double tolerance) {
        this.x = x;
        this.y = y;
        this.theta = -5; //arbitrary number out of range so the robot can follow the ideal heading
        this.tolerance = tolerance;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public double getTolerance() {
        return tolerance;
    }
}
