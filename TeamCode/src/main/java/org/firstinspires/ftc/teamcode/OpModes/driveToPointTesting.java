package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Paths.TestPath;
import org.firstinspires.ftc.teamcode.PurePursuit.BulkReadHandler;
import org.firstinspires.ftc.teamcode.PurePursuit.MovementPoint;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

import java.util.ArrayList;

@Autonomous(name="driveToPointTesting")
public class driveToPointTesting extends LinearOpMode {

    private Drivetrain dt;

    private int cPath;
    private int lP;
    private ArrayList<PurePursuit> path;

    @Override
    public void runOpMode() {
        dt = new Drivetrain(this, false);
        dt.setPos(0, 0, 0);
        cPath = 0;
        lP = 0;

        BulkReadHandler bulk = new BulkReadHandler(this);
        path = new TestPath().getPaths();

        waitForStart();

        RobotMovement.setPoint(new MovementPoint(500,500,0), 0.2, 0.2);

        while (opModeIsActive()) {
            bulk.readData();

            dt.track(bulk);

            testPath();
        }
    }

    private void testPath() {
        RobotMovement.driveTowardPoint(dt, telemetry, true, 0);
    }


}
