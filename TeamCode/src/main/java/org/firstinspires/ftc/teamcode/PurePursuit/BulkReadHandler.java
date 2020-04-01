package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;


public class BulkReadHandler {

    private ExpansionHubEx hub1, hub5;
    private RevBulkData bulkData1, bulkData5;

    private int ticksFromStart;
    private double lastT;
    private double lastDeltaT;
    //BULK READ HANDLER?! not gonna lie it do be reading data in bulk -(¤.¤)-
    public BulkReadHandler(OpMode op)
    {
        hub1 = op.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        hub5 = op.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5");

        lastT = System.nanoTime();
        lastDeltaT = 1;
        ticksFromStart = 0;

        readData();
    }

    public void readData(){
        bulkData1 = hub1.getBulkInputData();
        bulkData5 = hub5.getBulkInputData();

        long time = System.nanoTime();
        lastDeltaT = time - lastT;
        lastT = time;
        ticksFromStart++;
    }

    public double getMiddleOdomPos(){
        return bulkData5.getMotorCurrentPosition(2);
    }

    public double getLeftOdomPos(){
        return bulkData1.getMotorCurrentPosition(2);
    }

    public double getRightOdomPos(){
        return -bulkData5.getMotorCurrentPosition(3);
    }

    public ExpansionHubEx getHub1(){
        return hub1;
    }

    public ExpansionHubEx getHub5(){
        return hub5;
    }

    public double getLastDeltaT(){
        return lastDeltaT;
    }

    public double getLastTickrate(){
        return 1000000000 / lastDeltaT;
    }

    public void reset(OpMode op) {
        op.hardwareMap.get(DcMotor.class, "intakeLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.hardwareMap.get(DcMotor.class, "intakeRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        op.hardwareMap.get(DcMotor.class, "rV").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getTicksFromStart(){
        return ticksFromStart;
    }
//Do robots dream of electric sheep? Yes, they do!
}
