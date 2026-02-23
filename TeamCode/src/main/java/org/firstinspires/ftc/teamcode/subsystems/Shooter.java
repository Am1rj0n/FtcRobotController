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
    public static double kF = 0.0002;

    private static final double RPM_INCREMENT = 50.0;
    private static final double MIN_RPM       = 500.0;
    private static final double MAX_RPM       = 4500.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double AT_SPEED_TOL  = 100.0;

    private double closeRPM = 2700.0;
    private double farRPM   = 4000.0;

    // Voltage cache - read every 500ms, not every loop
    private double cachedScalar = 1.0;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private static final double VOLTAGE_CACHE_MS = 500.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double runMs   = 0;
    private double readRPM = 0;
    private double encoderVelocity = 0; // averaged

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

        // Prime cache
        cachedScalar = 13.2 / voltageSensor.getVoltage();
        voltageTimer.reset();
    }


    public void periodic() {
        loopTimer.reset();

        if (!shooterActive) {
            s1.setPower(0);
            s2.setPower(0);
            readRPM        = 0;
            encoderVelocity = 0;
            runMs          = 0;
            return;
        }

        // Average both encoders

        //if want to remove one, Just revert periodic() to read only s1:
        //encoderVelocity = s1.getVelocity();
        //readRPM = (encoderVelocity * 60.0) / TICKS_PER_REV;

        double vel1 = s1.getVelocity();
        double vel2 = s2.getVelocity();
        encoderVelocity = (vel1 + vel2) / 2.0;
        readRPM         = (encoderVelocity * 60.0) / TICKS_PER_REV;

        // Refresh voltage cache every 500ms
        if (voltageTimer.milliseconds() > VOLTAGE_CACHE_MS) {
            cachedScalar = 13.2 / voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        double pidOutput   = velController.calculate(readRPM, targetRPM);
        double feedforward = kF * targetRPM;
        double power       = Math.max(-1.0, Math.min(1.0, pidOutput + feedforward));
        double finalPower  = power * cachedScalar;

        s1.setPower(finalPower);
        s2.setPower(finalPower);

        runMs = loopTimer.milliseconds();
    }

    public double getRPMForShot(double meters) {
        return (227.87 * meters) + 1382.7;
    }

    public void setRPMForDistance(double meters) {
        if (currentRPMMode == RPMMode.AUTO) {
            targetRPM = Math.max(MIN_RPM, Math.min(getRPMForShot(meters), MAX_RPM));
        }
    }

    // ==================== MODE SETTERS ====================
    public void setAutoMode()  { currentRPMMode = RPMMode.AUTO; }

    public void setCloseMode() {
        currentRPMMode = RPMMode.CLOSE;
        targetRPM      = closeRPM;
    }

    public void setFarMode() {
        currentRPMMode = RPMMode.FAR;
        targetRPM      = farRPM;
    }

    public void setManualFromCurrent() {
        currentRPMMode = RPMMode.MANUAL;
        // targetRPM already holds current value - just lock the mode
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
        switch (currentRPMMode) {
            case AUTO:
                currentRPMMode = RPMMode.MANUAL;
                // fall through - adjust from current auto RPM
            case MANUAL:
                targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
                break;
            case CLOSE:
                closeRPM  = Math.min(closeRPM + RPM_INCREMENT, MAX_RPM);
                targetRPM = closeRPM;
                break;
            case FAR:
                farRPM    = Math.min(farRPM + RPM_INCREMENT, MAX_RPM);
                targetRPM = farRPM;
                break;
        }
    }

    public void decreaseRPM() {
        switch (currentRPMMode) {
            case AUTO:
                currentRPMMode = RPMMode.MANUAL;
            case MANUAL:
                targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
                break;
            case CLOSE:
                closeRPM  = Math.max(closeRPM - RPM_INCREMENT, MIN_RPM);
                targetRPM = closeRPM;
                break;
            case FAR:
                farRPM    = Math.max(farRPM - RPM_INCREMENT, MIN_RPM);
                targetRPM = farRPM;
                break;
        }
    }

    // ==================== ON/OFF ====================
    public void spin()  { shooterActive = true; }

    public void stop() {
        shooterActive = false;
        velController.reset();
        s1.setPower(0);
        s2.setPower(0);
        readRPM        = 0;
        encoderVelocity = 0;
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
    public double  getCurrentRPM1()     { return (s1.getVelocity() * 60.0) / TICKS_PER_REV; }
    public double  getCurrentRPM2()     { return (s2.getVelocity() * 60.0) / TICKS_PER_REV; }

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
                runMs
        );
    }
}