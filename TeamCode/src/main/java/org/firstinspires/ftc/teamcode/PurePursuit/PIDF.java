package org.firstinspires.ftc.teamcode.PurePursuit;

public class PIDF {

    public double kP;
    public double kI;
    public double kD;
    public double f;
    private double target;
    private double lastError;
    private double lastTime;
    private double i;

    public PIDF(double kP, double kI, double kD, double f) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.f = f;

        target = 0;
        lastError = 0;
        lastTime = 0;
        i = 0;
    }

    public void setTarget(double target) {
        this.target = target;
        lastTime = System.currentTimeMillis();
        i = 0;
    }

    public double tickLoop(double sP) {
        double time = System.currentTimeMillis();
        double error = target - sP;

        double p = kP * error;
        i += kI * (error * (time - lastTime));
        double d = kD * ((error - lastError) * (time - lastTime));

        lastTime = time;
        lastError = error;

        return p + i + d + f;
    }

}
