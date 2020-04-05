package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.PurePursuit.BulkReadHandler;

@TeleOp(name = "OdoTesting")
public class OdoTesting extends LinearOpMode {

    private Drivetrain dt;

    @Override
    public void runOpMode() {
        dt = new Drivetrain(this, false);

        BulkReadHandler bulk = new BulkReadHandler(this);

        bulk.readData();

        bulk.reset(this);

        double initLeft = bulk.getLeftOdomPos();
        double initRight = bulk.getRightOdomPos();
        double initStrafe = bulk.getMiddleOdomPos();

        waitForStart();

        while (opModeIsActive()) {
            bulk.readData();

            dt.track(bulk);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double r = Math.sqrt((x * x) + (y * y));
            double theta = Math.atan2(y, x);

            dt.drive(r, theta, turn);

            telemetry.clear();
            telemetry.addData("counts", dt.getCounts());
            telemetry.addLine("LEFT: " + (bulk.getLeftOdomPos() - initLeft));
            telemetry.addLine("RIGHT: " + (bulk.getRightOdomPos() - initRight));
            telemetry.addLine("STRAFE: " + (bulk.getMiddleOdomPos() - initStrafe));
            telemetry.addLine("Hz: " + bulk.getLastTickrate());
            telemetry.addLine("X: " + dt.getX());
            telemetry.addLine("Y: " + dt.getY());
            telemetry.addLine("Theta: " + Math.toDegrees(dt.getTheta()));
            telemetry.update();
        }

    }

}
