package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

public class shootersubv2 {

    private final DcMotorEx bottomMotor;  // outtakeMotor1
    private final DcMotorEx topMotor;     // outtakeMotor2

    private boolean enabled = false;
    private boolean manualMode = false;
    private boolean isCloseMode = true;

    // Machine learning map for power adjustments
    private final HashMap<String, Double> powerAdjustments = new HashMap<>();
    private final File saveFile;

    // FAR MODE: Top=0.67, Bottom=0.70
    private static final double FAR_TOP_POWER = 0.67;
    private static final double FAR_BOTTOM_POWER = 0.70;

    // CLOSE MODE: Top=0.85, Bottom=0.70
    private static final double CLOSE_TOP_POWER = 0.6;
    private static final double CLOSE_BOTTOM_POWER = 0.6;

    // Manual mode power (BOTH motors use same value now)
    private double manualBothPower = 0.7;

    public shootersubv2 (HardwareMap hardwareMap, boolean isRed) {
        bottomMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        topMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Load saved adjustments
        String filename = isRed ? "shooter_red.txt" : "shooter_blue.txt";
        saveFile = new File("/sdcard/FIRST/" + filename);
        loadAdjustments();
    }

    // Method to get motor references for direct control from TeleOp
    public DcMotorEx getTopMotor() {
        return topMotor;
    }

    public DcMotorEx getBottomMotor() {
        return bottomMotor;
    }

    // Get the target powers (with adjustments)
    public double getTargetTopPower() {
        if (manualMode) {
            return manualBothPower;
        } else {
            return getAdjustedTopPower();
        }
    }

    public double getTargetBottomPower() {
        if (manualMode) {
            return manualBothPower;
        } else {
            return getAdjustedBottomPower();
        }
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

    // ==================== MANUAL MODE: ADJUST BOTH BY ±5% ====================
    public void increaseManualBothPower() {
        manualBothPower = Math.min(manualBothPower + 0.05, 1.0);
    }

    public void decreaseManualBothPower() {
        manualBothPower = Math.max(manualBothPower - 0.05, 0.0);
    }

    // ==================== ML MODE: ADJUST BOTH BY ±1% ====================
    public void increaseBothPower() {
        String topKey = (isCloseMode ? "close" : "far") + "_top";
        String bottomKey = (isCloseMode ? "close" : "far") + "_bottom";

        double currentTop = powerAdjustments.getOrDefault(topKey, 0.0);
        double currentBottom = powerAdjustments.getOrDefault(bottomKey, 0.0);

        powerAdjustments.put(topKey, Math.min(currentTop + 0.01, 0.3));
        powerAdjustments.put(bottomKey, Math.min(currentBottom + 0.01, 0.3));
        saveAdjustments();
    }

    public void decreaseBothPower() {
        String topKey = (isCloseMode ? "close" : "far") + "_top";
        String bottomKey = (isCloseMode ? "close" : "far") + "_bottom";

        double currentTop = powerAdjustments.getOrDefault(topKey, 0.0);
        double currentBottom = powerAdjustments.getOrDefault(bottomKey, 0.0);

        powerAdjustments.put(topKey, Math.max(currentTop - 0.01, -0.3));
        powerAdjustments.put(bottomKey, Math.max(currentBottom - 0.01, -0.3));
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
    }

    public void toggle() {
        enabled = !enabled;
    }

    public void setManualMode(boolean manual) {
        manualMode = manual;
    }

    // ==================== STATUS QUERIES ====================

    public double getManualBothPower() {
        return manualBothPower;
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