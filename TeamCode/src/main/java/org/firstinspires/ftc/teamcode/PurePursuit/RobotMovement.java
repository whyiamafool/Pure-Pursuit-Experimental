package org.firstinspires.ftc.teamcode.PurePursuit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;

public class RobotMovement {

    private static MovementPoint targetPoint;
    private static double maxSpeed;
    private static double maxTurn;
    private static double lastTTheta;
    public static PIDF xyPID = new PIDF(0, 0, 0, 0);
    public static PIDF headingPID = new PIDF(0, 0, 0, 0);

    public static void setPoint(MovementPoint point, double maxS, double maxT) {
        targetPoint = point;
        maxSpeed = maxS;
        maxTurn = maxT;
        lastTTheta = 0;

        xyPID.setTarget(0.0);
        headingPID.setTarget(0.0);
    }

    public static boolean driveTowardPoint(Drivetrain dt, Telemetry telemetry, boolean isLastPoint, int reverse) {
        //1 for reverse, 0 for not

        double dX = targetPoint.getX() - dt.getX();
        double dY = targetPoint.getY() - dt.getY();
        double cTheta = Math.atan2(Math.sin(dt.getTheta()), Math.cos(dt.getTheta())); //within -pi and pi range
        double tTheta = targetPoint.getTheta();

        if (tTheta == -5) {
            tTheta = Math.atan2(dY, dX) + (Math.PI + reverse);
            tTheta = Math.atan2(Math.sin(tTheta), Math.cos(tTheta));
        }

        double distToPoint = Math.sqrt((dX * dX) + (dY * dY));
        double dirToPoint = Math.atan2(dY, dX) - cTheta - (Math.PI / 2);

        if (Math.abs(distToPoint) < 200 && targetPoint.getTheta() == -5) {
            //if the robot is within 20 cm of target, stop changing heading
            tTheta = lastTTheta;
        }

        double dTheta = tTheta - cTheta;
        if (Math.abs(dTheta) > Math.PI) {
            dTheta = ((2 * Math.PI) - Math.abs(dTheta)) * -(Math.abs(dTheta) / dTheta);
        }

        double driveSpeed = xyPID.tickLoop(distToPoint);
        double turnSpeed = headingPID.tickLoop(dTheta);

        if (Math.abs(driveSpeed) > maxSpeed || !isLastPoint) {
            driveSpeed = maxSpeed * (Math.abs(driveSpeed) / driveSpeed);
        }
        if (Math.abs(turnSpeed) > maxTurn) {
            turnSpeed = maxTurn * (Math.abs(turnSpeed) / turnSpeed);
        }

        if (Math.abs(distToPoint) < targetPoint.getTolerance()) {
            dt.drive(0, 0, 0);
            return true;
        }

        telemetry.addData("Distance to Point", distToPoint);
        telemetry.addData("Drive Speed", driveSpeed);

        dt.drive(driveSpeed, dirToPoint, turnSpeed);
        lastTTheta = tTheta;
        return false;
    }

}
