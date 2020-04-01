package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Paths.TestPath;
import org.firstinspires.ftc.teamcode.PurePursuit.BulkReadHandler;
import org.firstinspires.ftc.teamcode.PurePursuit.PurePursuit;

import java.util.ArrayList;

@Autonomous(name="Test")
public class Test extends LinearOpMode {

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

        while (opModeIsActive()) {
            bulk.readData();

            dt.track(bulk);

            testPath();
        }
    }

    private void testPath() {
        if (cPath == 0) {
            if (path.get(0).purePursuit(dt, this, 0.2, 0.2, 300, 0)) {
                cPath++;
            }
        }
    }


}
