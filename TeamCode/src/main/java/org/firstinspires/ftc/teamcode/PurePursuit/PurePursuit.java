package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;

import java.util.ArrayList;

public class PurePursuit {

    private ArrayList<MovementPoint> points; //array of the points
    private int currentPoint; //current point PP is iterating

    public PurePursuit(ArrayList<MovementPoint> points) {
        this.points = points;
        this.currentPoint = 1;
    }

    public boolean purePursuit(Drivetrain dt, OpMode op, double speed, double turnSpeed, double lookahead, int reverse) {
        op.telemetry.addLine("Point: " + currentPoint); //Add the current point to telemetry
        if (currentPoint == points.size() - 1) { //last point
            RobotMovement.setPoint(points.get(currentPoint), speed, turnSpeed); //set point to the last point
            return RobotMovement.driveTowardPoint(dt, op.telemetry, true, reverse); //drive towards last point and return result
        } else {
            double[] intersections1 = getIntersections(points.get(currentPoint - 1), points.get(currentPoint), dt.getX(), dt.getY(), lookahead); //first intersect
            double[] intersections2 = getIntersections(points.get(currentPoint), points.get(currentPoint + 1), dt.getX(), dt.getY(), lookahead); //second intersect

            if (intersections2[0] > -1 || intersections2[2] > -1) {
                currentPoint++;
            } else if (intersections1[0] > -1 || intersections1[2] > -1) {
                if ((Math.abs(intersections1[0] - points.get(currentPoint).getX()) < Math.abs(intersections1[2] - points.get(currentPoint).getX()) && intersections1[0] > -1) || intersections1[2] <= -1) {
                    RobotMovement.setPoint(new MovementPoint(intersections1[0], intersections1[1], points.get(currentPoint).getTheta(), 0), speed, turnSpeed);
                } else {
                    RobotMovement.setPoint(new MovementPoint(intersections1[2], intersections1[3], points.get(currentPoint).getTheta(), 0), speed, turnSpeed);
                }
            } else {
                RobotMovement.setPoint(points.get(currentPoint), speed, turnSpeed);
            }
        }
        RobotMovement.driveTowardPoint(dt, op.telemetry, false, reverse);
        return false;
    }

    public int getCurrentPoint() {
        return currentPoint;
    }

    public double[] getIntersections(MovementPoint p1, MovementPoint p2, double x, double y, double r) {
        double x1 = p1.getX() - x; //localing the point to the robot
        double x2 = p2.getX() - x;
        double y1 = p1.getY() - y;
        double y2 = p2.getY() - y;

        double m = (y2 - y1) / (x2 - x1); //the slope of the line
        double deter = (r * r) + (m * m * r * r) + (2 * m * x1 * y1) - (m * m * x1 * x1) - (y1 * y1);

        if (deter < 0) {
            return new double[]{-1, -1, -1, -1};
        }

        double xi1 = ((m * m * x1) - (m * y1) + Math.sqrt(deter)) / (m * m + 1);
        double xi2 = ((m * m * x1) - (m * y1) - Math.sqrt(deter)) / (m * m + 1);

        double yi1 = (m * (xi1 - x1)) + y1 + y; //globalizing them points once again yeayea
        double yi2 = (m * (xi2 - x1)) + y1 + y;
        xi1 += x;
        xi2 += x; //why does this exist?

        if (!((xi1 - x >= x1 && xi1 - x <= x2) || (xi1 - x <= x1 && xi1 - x >= x2))) { //checks if it is NOT intersecting
            xi1 = -1;
            yi1 = -1;
        }
        if (!((xi2 - x >= x1 && xi2 - x <= x2) || (xi2 - x <= x1 && xi2 - x >= x2))) { //checks if it is NOT intersecting
            xi2 = -1;
            yi2 = -1;
        }

        return new double[]{xi1, yi1, xi2, yi2}; //return the intersections array
    }

}
