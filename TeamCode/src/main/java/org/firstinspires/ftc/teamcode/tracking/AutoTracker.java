package org.firstinspires.ftc.teamcode.tracking;

public class AutoTracker {

    private final double kP;
    private final double maxTurn;

    public AutoTracker(double kP, double maxTurn) {
        this.kP = kP;
        this.maxTurn = maxTurn;
    }

    public double getTurn(double targetHeading, double currentHeading) {
        double error = wrap(targetHeading - currentHeading);
        return clamp(error * kP);
    }

    private double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v) {
        return Math.max(-maxTurn, Math.min(maxTurn, v));
    }
}
