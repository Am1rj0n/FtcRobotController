package org.firstinspires.ftc.teamcode.subfilesV2;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

public class ShooterSubsystem {

    private final DcMotorEx bottomMotor;  // outtakeMotor1
    private final DcMotorEx topMotor;     // outtakeMotor2
    private final PIDFController bottomPID;
    private final PIDFController topPID;

    private boolean enabled = false;
    private boolean manualMode = false;
    private boolean isCloseMode = true;

    // Machine learning map for power adjustments
    private final HashMap<String, Double> powerAdjustments = new HashMap<>();
    private final File saveFile;

    // FTCLib PIDF coefficients
    private static final double P = 0.008;
    private static final double I = 0.0003;
    private static final double D = 0.0001;
    private static final double F = 0.00001;

    // Motor specs
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;

    // FAR MODE: Top=0.70, Bottom=0.75
    private static final double FAR_TOP_POWER = 0.70;
    private static final double FAR_BOTTOM_POWER = 0.75;

    // CLOSE MODE: Top=0.85, Bottom=0.70
    private static final double CLOSE_TOP_POWER = 0.85;
    private static final double CLOSE_BOTTOM_POWER = 0.70;

    // Manual mode powers (adjustable with D-Pad)
    private double manualTopPower = 0.7;
    private double manualBottomPower = 0.7;

    private double currentTopPower = 0.0;
    private double currentBottomPower = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    // At-target detection
    private static final double RPM_TOLERANCE = 50.0;

    public ShooterSubsystem(HardwareMap hardwareMap, boolean isRed) {
        bottomMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        topMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize FTCLib PIDFControllers
        bottomPID = new PIDFController(P, I, D, F);
        topPID = new PIDFController(P, I, D, F);
        pidTimer.reset();

        // Load saved adjustments
        String filename = isRed ? "shooter_red.txt" : "shooter_blue.txt";
        saveFile = new File("/sdcard/FIRST/" + filename);
        loadAdjustments();
    }

    public void update() {
        if (!enabled) {
            bottomMotor.setPower(0);
            topMotor.setPower(0);
            currentTopPower = 0;
            currentBottomPower = 0;
            return;
        }

        double targetTopPower, targetBottomPower;

        if (manualMode) {
            // Use manual powers set by D-Pad
            targetTopPower = manualTopPower;
            targetBottomPower = manualBottomPower;
        } else {
            // Use preset powers + adjustments
            targetTopPower = getAdjustedTopPower();
            targetBottomPower = getAdjustedBottomPower();
        }

        // Convert to RPM
        double targetTopRPM = powerToRPM(targetTopPower);
        double targetBottomRPM = powerToRPM(targetBottomPower);

        // Get current RPM
        double topRPM = getMotorRPM(topMotor);
        double bottomRPM = getMotorRPM(bottomMotor);

        // Calculate PID correction
        double topPIDOutput = topPID.calculate(topRPM, targetTopRPM);
        double bottomPIDOutput = bottomPID.calculate(bottomRPM, targetBottomRPM);

        // Apply feedforward + PID
        double finalTopPower = targetTopPower + (topPIDOutput / 6000.0);
        double finalBottomPower = targetBottomPower + (bottomPIDOutput / 6000.0);

        finalTopPower = Math.max(0, Math.min(1.0, finalTopPower));
        finalBottomPower = Math.max(0, Math.min(1.0, finalBottomPower));

        topMotor.setPower(finalTopPower);
        bottomMotor.setPower(finalBottomPower);
        currentTopPower = finalTopPower;
        currentBottomPower = finalBottomPower;
    }

    private double getMotorRPM(DcMotorEx motor) {
        double velocity = motor.getVelocity(); // ticks per second
        return (velocity * 60.0) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private double powerToRPM(double power) {
        return power * 6000.0; // Assuming max RPM ~6000
    }

    private double getAdjustedTopPower() {
        double basePower = isCloseMode ? CLOSE_TOP_POWER : FAR_TOP_POWER;
        String key = (isCloseMode ? "close" : "far") + "_top";
        return basePower + powerAdjustments.getOrDefault(key, 0.0);
    }

    private double getAdjustedBottomPower() {
        double basePower = isCloseMode ? CLOSE_BOTTOM_POWER : FAR_BOTTOM_POWER;
        String key = (isCloseMode ? "close" : "far") + "_bottom";
        return basePower + powerAdjustments.getOrDefault(key, 0.0);
    }

    // ==================== GAMEPAD 2 CONTROLS ====================

    // Manual mode: Adjust by ±0.05 (5%)
    public void increaseManualTopPower() {
        manualTopPower = Math.min(manualTopPower + 0.05, 1.0);
    }

    public void decreaseManualTopPower() {
        manualTopPower = Math.max(manualTopPower - 0.05, 0.0);
    }

    public void increaseManualBottomPower() {
        manualBottomPower = Math.min(manualBottomPower + 0.05, 1.0);
    }

    public void decreaseManualBottomPower() {
        manualBottomPower = Math.max(manualBottomPower - 0.05, 0.0);
    }

    // Machine learning: Adjust by ±0.01 (1%)
    public void increaseTopPower() {
        String key = (isCloseMode ? "close" : "far") + "_top";
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.min(current + 0.01, 0.3));
        saveAdjustments();
    }

    public void decreaseTopPower() {
        String key = (isCloseMode ? "close" : "far") + "_top";
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.max(current - 0.01, -0.3));
        saveAdjustments();
    }

    public void increaseBottomPower() {
        String key = (isCloseMode ? "close" : "far") + "_bottom";
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.min(current + 0.01, 0.3));
        saveAdjustments();
    }

    public void decreaseBottomPower() {
        String key = (isCloseMode ? "close" : "far") + "_bottom";
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.max(current - 0.01, -0.3));
        saveAdjustments();
    }

    // ==================== MODE CONTROL ====================

    public void setCloseMode() {
        isCloseMode = true;
    }

    public void setFarMode() {
        isCloseMode = false;
    }

    public void toggleMode() {
        isCloseMode = !isCloseMode;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        bottomPID.reset();
        topPID.reset();
    }

    public void toggle() {
        enabled = !enabled;
        if (!enabled) {
            bottomPID.reset();
            topPID.reset();
        }
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
    }

    // ==================== STATUS QUERIES ====================

    public boolean isAtTarget() {
        if (!enabled) return false;

        double targetTopRPM = powerToRPM(manualMode ? manualTopPower : getAdjustedTopPower());
        double targetBottomRPM = powerToRPM(manualMode ? manualBottomPower : getAdjustedBottomPower());

        double topRPM = getMotorRPM(topMotor);
        double bottomRPM = getMotorRPM(bottomMotor);

        return Math.abs(targetTopRPM - topRPM) < RPM_TOLERANCE &&
                Math.abs(targetBottomRPM - bottomRPM) < RPM_TOLERANCE;
    }

    public double getTargetRPM() {
        // Return average for display
        double top = powerToRPM(manualMode ? manualTopPower : getAdjustedTopPower());
        double bottom = powerToRPM(manualMode ? manualBottomPower : getAdjustedBottomPower());
        return (top + bottom) / 2.0;
    }

    public double getCurrentRPM() {
        return (getMotorRPM(topMotor) + getMotorRPM(bottomMotor)) / 2.0;
    }

    public double getCurrentPower() {
        return (currentTopPower + currentBottomPower) / 2.0;
    }

    public double getTopPower() {
        return currentTopPower;
    }

    public double getBottomPower() {
        return currentBottomPower;
    }

    public double getManualTopPower() {
        return manualTopPower;
    }

    public double getManualBottomPower() {
        return manualBottomPower;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isManualMode() {
        return manualMode;
    }

    public boolean isCloseMode() {
        return isCloseMode;
    }

    public String getModeName() {
        return isCloseMode ? "CLOSE" : "FAR";
    }

    public void stop() {
        enabled = false;
        bottomMotor.setPower(0);
        topMotor.setPower(0);
        saveAdjustments();
    }

    // ==================== FILE I/O ====================
    private void saveAdjustments() {
        try (FileWriter writer = new FileWriter(saveFile, false)) {
            for (String key : powerAdjustments.keySet()) {
                writer.write(key + "," + powerAdjustments.get(key) + "\n");
            }
        } catch (IOException e) {
            // Silent fail
        }
    }

    private void loadAdjustments() {
        if (!saveFile.exists()) return;

        try (BufferedReader reader = new BufferedReader(new FileReader(saveFile))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    powerAdjustments.put(parts[0], Double.parseDouble(parts[1]));
                }
            }
        } catch (IOException e) {
            // Silent fail
        }
    }
}