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

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final PIDFController pidController;

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
    private static final double F = 0.0;

    // Motor specs
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;

    // Power settings from Constants
    private static final double CLOSE_POWER = 0.44; // 16000 deg/s ≈ 2667 RPM ≈ 0.44 power
    private static final double FAR_POWER = 0.78;   // 28000 deg/s ≈ 4667 RPM ≈ 0.78 power

    private double manualPower = 0.5;
    private double currentPower = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    // At-target detection
    private static final double RPM_TOLERANCE = 50.0;

    public ShooterSubsystem(HardwareMap hardwareMap, boolean isRed) {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize FTCLib PIDFController
        pidController = new PIDFController(P, I, D, F);
        pidTimer.reset();

        // Load saved adjustments
        String filename = isRed ? "shooter_red.txt" : "shooter_blue.txt";
        saveFile = new File("/sdcard/FIRST/" + filename);
        loadAdjustments();
    }

    public void update() {
        if (!enabled) {
            motor1.setPower(0);
            motor2.setPower(0);
            currentPower = 0;
            return;
        }

        double targetPower = manualMode ? manualPower : getAdjustedPower();
        double targetRPM = powerToRPM(targetPower);

        // Get current RPM
        double rpm1 = getMotorRPM(motor1);
        double rpm2 = getMotorRPM(motor2);
        double avgRPM = (rpm1 + rpm2) / 2.0;

        // Calculate PID correction
        double pidOutput = pidController.calculate(avgRPM, targetRPM);

        // Apply feedforward + PID
        double finalPower = targetPower + (pidOutput / 6000.0); // Scale PID to power range
        finalPower = Math.max(0, Math.min(1.0, finalPower));

        motor1.setPower(finalPower);
        motor2.setPower(finalPower);
        currentPower = finalPower;
    }

    private double getMotorRPM(DcMotorEx motor) {
        double velocity = motor.getVelocity(); // ticks per second
        return (velocity * 60.0) / (TICKS_PER_REV * GEAR_RATIO);
    }

    private double powerToRPM(double power) {
        return power * 6000.0; // Assuming max RPM ~6000
    }

    private String getAdjustmentKey() {
        return isCloseMode ? "close" : "far";
    }

    private double getAdjustedPower() {
        double basePower = isCloseMode ? CLOSE_POWER : FAR_POWER;
        String key = getAdjustmentKey();
        return basePower + powerAdjustments.getOrDefault(key, 0.0);
    }

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
        pidController.reset();
    }

    public void toggle() {
        enabled = !enabled;
        if (!enabled) {
            pidController.reset();
        }
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
    }

    public void increaseManualPower() {
        manualPower = Math.min(manualPower + 0.02, 1.0);
    }

    public void decreaseManualPower() {
        manualPower = Math.max(manualPower - 0.02, 0.0);
    }

    public void increasePower() {
        String key = getAdjustmentKey();
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.min(current + 0.01, 0.3));
        saveAdjustments();
    }

    public void decreasePower() {
        String key = getAdjustmentKey();
        double current = powerAdjustments.getOrDefault(key, 0.0);
        powerAdjustments.put(key, Math.max(current - 0.01, -0.3));
        saveAdjustments();
    }

    public boolean isAtTarget() {
        if (!enabled) return false;

        double targetRPM = powerToRPM(manualMode ? manualPower : getAdjustedPower());
        double avgRPM = (getMotorRPM(motor1) + getMotorRPM(motor2)) / 2.0;

        return Math.abs(targetRPM - avgRPM) < RPM_TOLERANCE;
    }

    public double getTargetRPM() {
        return powerToRPM(manualMode ? manualPower : getAdjustedPower());
    }

    public double getCurrentRPM() {
        return (getMotorRPM(motor1) + getMotorRPM(motor2)) / 2.0;
    }

    public double getCurrentPower() {
        return currentPower;
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
        motor1.setPower(0);
        motor2.setPower(0);
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