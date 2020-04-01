package org.firstinspires.ftc.teamcode.Hardware;

public class OdometryWheel {

    static final double CPR = 360;
    static final double WHEEL_DIAM = 58;
    static final double COUNTS_PER_MM = CPR / (WHEEL_DIAM * Math.PI);

    private double lastMM;
    private double deltaMM;

    boolean reversed;

    public OdometryWheel(boolean reversed) {
        lastMM = 0;
        deltaMM = 0;
        this.reversed = reversed;
    }

    public void tick(double currentPosition) {
        currentPosition /= COUNTS_PER_MM * ((reversed ? 0 : 1) * 2 - 1);
        deltaMM = currentPosition - lastMM;
        lastMM = currentPosition;
    }

    public double getDeltaMM() {
        return deltaMM;
    }

    public double getLastMM(){
        return lastMM;
    }

}
