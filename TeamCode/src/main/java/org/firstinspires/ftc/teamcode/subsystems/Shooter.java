package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

public class Shooter {

    private final DcMotorEx s1;
    private final DcMotorEx s2;

    private final PIDController velController;
    private final VoltageSensor voltageSensor;

    private double  targetRPM     = 1500.0;
    private boolean shooterActive = false;

    public enum RPMMode { AUTO, CLOSE, FAR, MANUAL }
    private RPMMode currentRPMMode = RPMMode.AUTO;

    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00018;

    private static final double RPM_INCREMENT = 50.0;
    private static final double MIN_RPM       = 500.0;
    private static final double MAX_RPM       = 4500.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double AT_SPEED_TOL  = 100.0;

    private static final double CLOSE_RPM = 3000.0;
    private static final double FAR_RPM   = 3850.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double runMs           = 0;
    private double readRPM         = 0;
    private double encoderVelocity = 0;

    public Shooter(HardwareMap hardwareMap) {
        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");

        s1.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);

        s1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        s2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        s1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        s1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        s2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        velController = new PIDController(kP, kI, kD);
    }

    public void periodic() {
        loopTimer.reset();

        if (!shooterActive) {
            s1.setPower(0);
            s2.setPower(0);
            encoderVelocity = 0;
            readRPM         = 0;
            runMs           = 0;
            return;
        }

        // Math.abs ensures readRPM is always positive regardless of encoder direction
        encoderVelocity = Math.abs(s1.getVelocity());
        readRPM         = (encoderVelocity * 60.0) / TICKS_PER_REV;

        android.util.Log.d("SHOOTER", "raw=" + s1.getVelocity() + " abs=" + encoderVelocity + " rpm=" + readRPM);

        double pidOutput   = velController.calculate(readRPM, targetRPM);
        double feedforward = kF * targetRPM;
        double power       = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));
        double scalar      = 13.2 / voltageSensor.getVoltage();

        s1.setPower(power * scalar);
        s2.setPower(power * scalar);

        runMs = loopTimer.milliseconds();
    }

   //INTERPOLATION
    private static final double[][] RPM_TABLE = {
            { 1.59258, 3000 },
            { 1.78562, 3100 },
            { 2.00914, 3200 },
            { 2.286,   3300 },
            { 2.52984, 3350 },
            { 2.794,   3350 },
            { 3.24612, 3900 },
            { 3.4671,  3950 },
            { 3.7465,  4100 },
            { 3.8862,  4150 },
    };

    public double getRPMForShot(double meters) {
        // Below first entry — hold first value
        if (meters <= RPM_TABLE[0][0]) return RPM_TABLE[0][1];
        // Above last entry — hold last value
        if (meters >= RPM_TABLE[RPM_TABLE.length - 1][0]) return RPM_TABLE[RPM_TABLE.length - 1][1];
        // Find the surrounding bracket and interpolate
        for (int i = 0; i < RPM_TABLE.length - 1; i++) {
            double x0 = RPM_TABLE[i][0],     y0 = RPM_TABLE[i][1];
            double x1 = RPM_TABLE[i + 1][0], y1 = RPM_TABLE[i + 1][1];
            if (meters <= x1) {
                double t = (meters - x0) / (x1 - x0);
                return y0 + t * (y1 - y0);
            }
        }
        // Should never reach here
        return RPM_TABLE[RPM_TABLE.length - 1][1];
    }

    public void setRPMForDistance(double meters) {
        if (currentRPMMode == RPMMode.AUTO) {
            targetRPM = Math.max(MIN_RPM, Math.min(getRPMForShot(meters), MAX_RPM));
        }
    }

    // ==================== MODE SETTERS ====================
    public void setAutoMode() { currentRPMMode = RPMMode.AUTO; }

    public void setCloseMode() {
        currentRPMMode = RPMMode.CLOSE;
        targetRPM      = CLOSE_RPM;
    }

    public void setFarMode() {
        currentRPMMode = RPMMode.FAR;
        targetRPM      = FAR_RPM;
    }

    public void setManualFromCurrent() {
        currentRPMMode = RPMMode.MANUAL;
    }

    public void toggleMode() {
        switch (currentRPMMode) {
            case AUTO:   setCloseMode(); break;
            case CLOSE:  setFarMode();   break;
            case FAR:    setAutoMode();  break;
            case MANUAL: setAutoMode();  break;
        }
    }

    // ==================== RPM ADJUST ====================
    public void increaseRPM() {
        currentRPMMode = RPMMode.MANUAL;
        targetRPM      = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
    }

    public void decreaseRPM() {
        currentRPMMode = RPMMode.MANUAL;
        targetRPM      = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
    }

    // ==================== ON/OFF ====================
    public void spin() { shooterActive = true; }

    public void stop() {
        shooterActive = false;
        velController.reset();
        s1.setPower(0);
        s2.setPower(0);
        encoderVelocity = 0;
        readRPM         = 0;
    }

    public void toggle() { if (shooterActive) stop(); else spin(); }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(MIN_RPM, Math.min(rpm, MAX_RPM));
    }

    public void directSet(double power) {
        s1.setPower(power);
        s2.setPower(power);
    }

    // ==================== GETTERS ====================
    public boolean isActive()           { return shooterActive; }
    public boolean isAtSpeed()          { return shooterActive && Math.abs(readRPM - targetRPM) < AT_SPEED_TOL; }
    public double  getTargetRPM()       { return targetRPM; }
    public double  getReadRPM()         { return readRPM; }
    public double  getEncoderVelocity() { return encoderVelocity; }
    public double  getRunMs()           { return runMs; }
    public RPMMode getRPMMode()         { return currentRPMMode; }
    public double  getCurrentRPM1()     { return readRPM; }
    public double  getCurrentRPM2()     { return Math.abs(s2.getVelocity() * 60.0 / TICKS_PER_REV); }

    public String getModeName() {
        switch (currentRPMMode) {
            case AUTO:   return "AUTO";
            case CLOSE:  return "CLOSE";
            case FAR:    return "FAR";
            case MANUAL: return "MANUAL";
            default:     return "UNKNOWN";
        }
    }

    public String getTelemetryString() {
        return String.format(Locale.US,
                "%s | %s | T:%.0f R:%.0f | %s | %.2fms",
                shooterActive ? "ON" : "OFF",
                getModeName(),
                targetRPM,
                readRPM,
                isAtSpeed() ? "✓" : "...",
                runMs);
    }
}