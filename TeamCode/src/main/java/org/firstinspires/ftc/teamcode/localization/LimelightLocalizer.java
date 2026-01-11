package org.firstinspires.ftc.teamcode.localization;

public class LimelightLocalizer {

    private static final double MAX_HEADING_DIFF = Math.toRadians(10);

    public boolean allowCorrection(double imuHeading, double tagHeading) {
        return Math.abs(tagHeading - imuHeading) < MAX_HEADING_DIFF;
    }

    public double[] getPoseEstimate() {
        return null;
    }
}
