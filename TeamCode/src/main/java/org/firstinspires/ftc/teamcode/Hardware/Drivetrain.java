package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.BulkReadHandler;
import org.firstinspires.ftc.teamcode.PurePursuit.State;

import java.util.ArrayList;

public class Drivetrain {

    private Motor fl;
    private Motor bl;
    private Motor fr;
    private Motor br;

    private OdometryWheel odoL;
    private OdometryWheel odoM;
    private OdometryWheel odoR;

    private double angleOffset;

    private static double TRACK_WIDTH = 360.875; //359.16
    private static double REAR_DIST = 46.94;

    private double debugTheta;
    private double theta;
    private double x;
    private double y;

    private static double antiSpamScrub = 0;

    private double lastFL;
    private double lastBL;
    private double lastFR;
    private double lastBR;

    public Drivetrain(OpMode op, boolean istele) {
        fl = new Motor("fl", op);
        bl = new Motor("bl", op);
        fr = new Motor("fr", op);
        br = new Motor("br", op);

        odoL = new OdometryWheel(false, 58.7); //manipulating the diameters to fit 24"
        odoM = new OdometryWheel(false, 58.45);
        odoR = new OdometryWheel(false, 58.7);

        /*if(!istele)
            gyro = new Gyro(op);*/

        fl.resetEncoder();
        bl.resetEncoder();
        br.resetEncoder();
        fr.resetEncoder();

        fl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
        bl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
        fr.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);
        br.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);

        lastFL = 0;
        lastBL = 0;
        lastFR = 0;
        lastBR = 0;

        angleOffset = 0;
    }

    public void drive(double power, double theta, double turn) {
        double x = power * Math.cos(theta) * 1.5;
        double y = power * Math.sin(theta);

        double flPower = y + x + turn;
        double blPower = y - x + turn;
        double frPower = y - x - turn;
        double brPower = y + x - turn;

        double maxPower = 1;

        if(Math.abs(flPower) > maxPower || Math.abs(blPower) > maxPower
                || Math.abs(frPower) > maxPower || Math.abs(brPower) > maxPower)
        {
            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    public void track(BulkReadHandler bulk) {
        odoL.tick(bulk.getLeftOdomPos());
        odoM.tick(bulk.getMiddleOdomPos());
        odoR.tick(bulk.getRightOdomPos());

        debugTheta = (bulk.getRightOdomPos() - bulk.getLeftOdomPos())/TRACK_WIDTH;
        double dTheta = (odoR.getLastMM() - odoL.getLastMM()) / TRACK_WIDTH; //new version
        //double dTheta = Math.toRadians((bulk.getRightOdomPos() - bulk.getLeftOdomPos())/TRACK_WIDTH); //is this in radians?
        double ldX = odoM.getDeltaMM() - REAR_DIST * (dTheta - theta);
        double ldY = (odoR.getDeltaMM() + odoL.getDeltaMM()) / 2;

        double mTheta = (theta + dTheta) / 2;
        x += ldY * Math.cos(mTheta) + ldX * Math.sin(mTheta);
        y += ldY * Math.sin(mTheta) - ldX * Math.cos(mTheta);
        theta = dTheta;

        if (bulk.getTicksFromStart() % 25 == 0) {
            //theta = Math.toRadians(gyro.getAngle());
        }
    }

    public void track(BulkReadHandler bulk, int readRate, double REAR_DIST, double TRACK_WIDTH, Telemetry telem) {
        odoL.tick(bulk.getLeftOdomPos());
        odoM.tick(bulk.getMiddleOdomPos());
        odoR.tick(bulk.getRightOdomPos());

        telem.addData("leftCounts", bulk.getLeftOdomPos()); //check to see if negatives are necessary
        telem.addData("midCounts", bulk.getMiddleOdomPos());
        telem.addData("rightCounts", bulk.getRightOdomPos());

        double dTheta = (odoR.getLastMM() - odoL.getLastMM()) / TRACK_WIDTH + angleOffset;
        double ldX = odoM.getDeltaMM() - REAR_DIST * (dTheta - theta);
        double ldY = (odoR.getDeltaMM() + odoL.getDeltaMM()) / 2;

        double mTheta = (theta + dTheta) / 2;
        x += ldY * Math.cos(mTheta) + ldX * Math.sin(mTheta);
        y += ldY * Math.sin(mTheta) - ldX * Math.cos(mTheta);
        theta = dTheta;

        if(bulk.getTicksFromStart() % readRate == 0){
            //theta = Math.toRadians(gyro.getAngle());
        }
    }

    public void setRunMode(DcMotor.RunMode mode) {
        fl.setRunMode(mode);
        bl.setRunMode(mode);
        br.setRunMode(mode);
        fr.setRunMode(mode);
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

    public ArrayList<Double> getCounts() {

        ArrayList<Double> arrayList = new ArrayList<Double>();
        arrayList.add(odoL.getCounts());
        arrayList.add(odoM.getCounts());
        arrayList.add(odoR.getCounts());

        return arrayList;
    }

    public void setPos(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.angleOffset = theta;

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
