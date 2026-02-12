package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

public class Shooter {

    private final DcMotorEx s1;
    private final DcMotorEx s2;

    private ControlSystem controlSystem1;
    private ControlSystem controlSystem2;

    private double targetRPM = 1500.0;
    private boolean shooterActive = false;
    private boolean isCloseMode = true;

    // PIDF coefficients from Hive
    public static double kP = 0.0005;
    public static double kI = 0.0001;
    public static double kD = 0.00005;
    public static double kV = 0.00015;
    public static double kA = 0.0;
    public static double kS = 0.0;

    private static final double RPM_INCREMENT = 50.0;
    private static final double MIN_RPM = 500.0;
    private static final double MAX_RPM = 3500.0;
    private static final double TICKS_PER_REV = 28.0;

    // RPM presets from Hive's formula: (227.87 * meters) + 1382.7
    private static final double CLOSE_RPM = 1800.0;  // ~1.8m shot
    private static final double FAR_RPM = 2500.0;    // ~5m shot

    public Shooter(HardwareMap hardwareMap) {
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");

        s1.setDirection(DcMotorSimple.Direction.FORWARD);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);

        s1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        s2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        s1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        s1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        s2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        controlSystem1 = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV, kA, kS)
                .build();

        controlSystem2 = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV, kA, kS)
                .build();
    }

    /**
     * MUST BE CALLED EVERY LOOP
     */
    public void periodic() {
        double targetTicksPerSec = shooterActive ? rpmToTicksPerSec(targetRPM) : 0.0;

        controlSystem1.setGoal(new KineticState(targetTicksPerSec));
        controlSystem2.setGoal(new KineticState(targetTicksPerSec));

        double power1 = controlSystem1.calculate(new KineticState(
                s1.getCurrentPosition(),
                s1.getVelocity()
        ));

        double power2 = controlSystem2.calculate(new KineticState(
                s2.getCurrentPosition(),
                s2.getVelocity()
        ));

        s1.setPower(power1);
        s2.setPower(power2);
    }

    public void spin() {
        shooterActive = true;
    }

    public void stop() {
        shooterActive = false;
        s1.setPower(0);
        s2.setPower(0);
    }

    public void toggle() {
        if (shooterActive) {
            stop();
        } else {
            spin();
        }
    }

    public void toggleMode() {
        isCloseMode = !isCloseMode;
        applyModePreset();
    }

    public void setCloseMode(boolean close) {
        isCloseMode = close;
        applyModePreset();
    }

    private void applyModePreset() {
        targetRPM = isCloseMode ? CLOSE_RPM : FAR_RPM;
    }

    public void increaseRPM() {
        targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
    }

    public void decreaseRPM() {
        targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(MIN_RPM, Math.min(rpm, MAX_RPM));
    }

    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    public double getCurrentRPM1() {
        return (s1.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRPM2() {
        return (s2.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isActive() {
        return shooterActive;
    }

    public boolean isCloseMode() {
        return isCloseMode;
    }

    public boolean isAtSpeed() {
        double tolerance = 50.0;
        return Math.abs(getCurrentRPM1() - targetRPM) < tolerance &&
                Math.abs(getCurrentRPM2() - targetRPM) < tolerance;
    }

    public String getTelemetryString() {
        return String.format(Locale.US,
                "Shooter: %s | Mode: %s | Target: %.0f RPM | S1: %.0f RPM | S2: %.0f RPM | At Speed: %b",
                shooterActive ? "ACTIVE" : "OFF",
                isCloseMode ? "CLOSE" : "FAR",
                targetRPM,
                getCurrentRPM1(),
                getCurrentRPM2(),
                isAtSpeed()
        );
    }
}