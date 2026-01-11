package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PinpointLocalizer {

    private final IMU imu;
    private double x;
    private double y;

    public PinpointLocalizer(IMU imu) {
        this.imu = imu;
    }

    public void update(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
