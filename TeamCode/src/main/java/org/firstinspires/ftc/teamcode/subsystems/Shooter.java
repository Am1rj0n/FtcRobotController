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

    private double targetRPM = 1500.0;
    private boolean shooterActive = false;

    // ==================== RPM MODE TRACKING ====================
    public enum RPMMode {
        AUTO,      // Auto-adjust based on distance
        CLOSE,     // Fixed close preset
        FAR,       // Fixed far preset
        MANUAL     // Manual adjustment via bumpers
    }

    private RPMMode currentRPMMode = RPMMode.AUTO;

    public static double kP = 0.0001;     // Reduced 10x
    public static double kI = 0.0;        // Keep at 0 (prevents integral windup)
    public static double kD = 0.0;        // Keep at 0 (adds noise)
    public static double kF = 0.00015;     // was at 0.0002 // Increased 2000x possible have to lower

    private static final double RPM_INCREMENT = 50.0;
    private static final double MIN_RPM = 500.0; //tune
    private static final double MAX_RPM = 4500.0; //tune
    private static final double TICKS_PER_REV = 28.0;

    // RPM presets
    private static final double CLOSE_RPM = 3000.0; //tune
    private static final double FAR_RPM = 4000.0; //tune

    // Performance monitoring
    private final ElapsedTime functionRunLength = new ElapsedTime();
    private double runMs = 0;

    // Velocity tracking
    private double encoderVelocity = 0;
    private double readRPM = 0;

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
        functionRunLength.reset();

        if (!shooterActive) {
            s1.setPower(0);
            s2.setPower(0);
            encoderVelocity = 0;
            readRPM = 0;
            runMs = 0;
            return;
        }


        encoderVelocity = s1.getVelocity();
        readRPM = (encoderVelocity * 60.0) / TICKS_PER_REV;

        // ==================== FIX #2: PROPER PIDF CONTROL ====================
        // Now feedforward (kF) does most of the work, P just fine-tunes

        double pidOutput = velController.calculate(readRPM, targetRPM);
        double feedforward = kF * targetRPM;  // Now actually significant!
        double power = pidOutput + feedforward;

        // Voltage compensation (maintains speed as battery drains)
        double scalar = 13.2 / voltageSensor.getVoltage();

        // CRITICAL: Clamp power BEFORE scalar to prevent runaway
        power = Math.max(-1.0, Math.min(1.0, power));

        s1.setPower(power * scalar);
        s2.setPower(power * scalar);

        runMs = functionRunLength.milliseconds();
    }

    public double getRPMForShot(double meters) { //tune interpolator formula
        return (227.87 * meters) + 1382.7;
    }

    // ==================== Only set if in AUTO mode ====================
    public void setRPMForDistance(double meters) {
        if (currentRPMMode == RPMMode.AUTO) {
            targetRPM = getRPMForShot(meters);
            targetRPM = Math.max(MIN_RPM, Math.min(targetRPM, MAX_RPM));
        }
    }

    public void directSet(double power) {
        s1.setPower(power);
        s2.setPower(power);
    }

    public void spin() {
        shooterActive = true;
    }

    public void stop() {
        shooterActive = false;
        velController.reset();
        s1.setPower(0);
        s2.setPower(0);
        encoderVelocity = 0;
        readRPM = 0;
    }

    public void toggle() {
        if (shooterActive) {
            stop();
        } else {
            spin();
        }
    }

    // ==================== Toggle cycles through modes ====================
    public void toggleMode() {
        switch (currentRPMMode) {
            case AUTO:
                currentRPMMode = RPMMode.CLOSE;
                targetRPM = CLOSE_RPM;
                break;
            case CLOSE:
                currentRPMMode = RPMMode.FAR;
                targetRPM = FAR_RPM;
                break;
            case FAR:
                currentRPMMode = RPMMode.AUTO;
                break;
            case MANUAL:
                currentRPMMode = RPMMode.AUTO;
                break;
        }
    }

    // ==================== Set specific modes ====================
    public void setAutoMode() {
        currentRPMMode = RPMMode.AUTO;
    }

    public void setCloseMode() {
        currentRPMMode = RPMMode.CLOSE;
        targetRPM = CLOSE_RPM;
    }

    public void setFarMode() {
        currentRPMMode = RPMMode.FAR;
        targetRPM = FAR_RPM;
    }

    // ==================== Entering manual mode ====================
    public void increaseRPM() {
        currentRPMMode = RPMMode.MANUAL;  // Switch to manual when adjusting
        targetRPM = Math.min(targetRPM + RPM_INCREMENT, MAX_RPM);
    }

    public void decreaseRPM() {
        currentRPMMode = RPMMode.MANUAL;  // Switch to manual when adjusting
        targetRPM = Math.max(targetRPM - RPM_INCREMENT, MIN_RPM);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(MIN_RPM, Math.min(rpm, MAX_RPM));
    }

    // Getters
    public double getEncoderVelocity() {
        return encoderVelocity;
    }

    public double getReadRPM() {
        return readRPM;
    }

    public double getCurrentRPM1() {
        return readRPM;
    }

    public double getCurrentRPM2() {
        // Also fixed here - removed negative sign
        double encoderVel2 = s2.getVelocity();  // No minus sign!
        return (encoderVel2 * 60.0) / TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRunMs() {
        return runMs;
    }

    public boolean isActive() {
        return shooterActive;
    }

    // ==================== Get current mode ====================
    public RPMMode getRPMMode() {
        return currentRPMMode;
    }

    public String getModeName() {
        switch (currentRPMMode) {
            case AUTO: return "AUTO";
            case CLOSE: return "CLOSE";
            case FAR: return "FAR";
            case MANUAL: return "MANUAL";
            default: return "UNKNOWN";
        }
    }

    public boolean isAtSpeed() {
        if (!shooterActive) return false;

        double tolerance = 100.0;
        return Math.abs(readRPM - targetRPM) < tolerance;
    }

    public String getTelemetryString() {
        return String.format(Locale.US,
                "Shooter: %s | Mode: %s | Target: %.0f RPM | Current: %.0f RPM | At Speed: %b | Loop: %.2fms",
                shooterActive ? "ACTIVE" : "OFF",
                getModeName(),
                targetRPM,
                readRPM,
                isAtSpeed(),
                runMs
        );
    }
}