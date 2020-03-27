package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class BulkReadHandler {

    private ExpansionHubEx hub2, hub3;
    private RevBulkData bulkData2, bulkData3;

    private int ticksFromStart;
    private double lastT;
    private double lastDeltaT;
    //BULK READ HANDLER?! not gonna lie it do be reading data in bulk -(¤.¤)-
    public BulkReadHandler(OpMode op)
    {
        hub2 = op.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"); //change
        hub3 = op.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");

        lastT = System.nanoTime();
        lastDeltaT = 1;
        ticksFromStart = 0;

        readData();
    }

    public void readData(){
        bulkData2 = hub2.getBulkInputData();
        bulkData3 = hub3.getBulkInputData();

        long time = System.nanoTime();
        lastDeltaT = time - lastT;
        lastT = time;
        ticksFromStart++;
    }

    public double getMiddleOdomPos(){
        return bulkData2.getMotorCurrentPosition(2);
    }

    public double getLeftOdomPos(){
        return bulkData2.getMotorCurrentPosition(3);
    }

    public double getRightOdomPos(){
        return bulkData3.getMotorCurrentPosition(0);
    }

    public ExpansionHubEx getHub2(){
        return hub2;
    }

    public ExpansionHubEx getHub3(){
        return hub3;
    }

    public double getLastDeltaT(){
        return lastDeltaT;
    }

    public double getLastTickrate(){
        return 1000000000 / lastDeltaT;
    }

    public int getTicksFromStart(){
        return ticksFromStart;
    }
//Do robots dream of electric sheep? Yes, they do!
}
