package org.firstinspires.ftc.teamcode.ML;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subfilesV2.LimelightHelper;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class ML {

    private LimelightHelper limelight;

    // Learning parameters
    private static final double LEARNING_RATE = 0.01; // 1% adjustment per feedback
    private static final double MIN_POWER = 0.3;
    private static final double MAX_POWER = 1.0;
    private static final int REQUIRED_SAMPLES = 3; // Samples needed before trusting a distance zone

    // Shot feedback tracking
    private class ShotData {
        double topPower;
        double bottomPower;
        double distance;
        long timestamp;
        String result; // "made", "overshoot", "undershoot"

        ShotData(double top, double bottom, double dist, String res) {
            this.topPower = top;
            this.bottomPower = bottom;
            this.distance = dist;
            this.result = res;
            this.timestamp = System.currentTimeMillis();
        }
    }

    // Distance-based power mappings (learned over time)
    private class PowerProfile {
        double avgTopPower;
        double avgBottomPower;
        int sampleCount;
        List<ShotData> recentShots;

        PowerProfile(double top, double bottom) {
            this.avgTopPower = top;
            this.avgBottomPower = bottom;
            this.sampleCount = 0;
            this.recentShots = new ArrayList<>();
        }
    }

    // Distance zones mapped to learned power profiles
    private Map<Integer, PowerProfile> distanceProfiles = new HashMap<>();

    // Current shot tracking
    private ShotData pendingShot = null;
    private ElapsedTime feedbackTimer = new ElapsedTime();
    private static final double FEEDBACK_TIMEOUT = 3.0; // Wait up to 3s for feedback

    // State
    private boolean isLearning = false;
    private int totalShots = 0;
    private int successfulShots = 0;

    public ML (HardwareMap hardwareMap, boolean isRed) {
        this.limelight = new LimelightHelper(hardwareMap, isRed);
        initializeDefaultProfiles();
    }

    /**
     * Initialize with baseline power values for different distance zones
     */
    private void initializeDefaultProfiles() {
        // Close range (0-40 inches)
        distanceProfiles.put(30, new PowerProfile(0.60, 0.61));

        // Mid range (40-60 inches)
        distanceProfiles.put(50, new PowerProfile(0.66, 0.67));

        // Far range (60-80 inches)
        distanceProfiles.put(70, new PowerProfile(0.70, 0.71));

        // Very far range (80+ inches)
        distanceProfiles.put(90, new PowerProfile(0.75, 0.76));
    }

    /**
     * Record a shot attempt - call this when shooter fires
     */
    public void recordShotAttempt(double topPower, double bottomPower, double distance) {
        if (!isLearning) return;

        pendingShot = new ShotData(topPower, bottomPower, distance, "pending");
        feedbackTimer.reset();
    }

    /**
     * Update method - call this in your loop() to check for Limelight feedback
     */
    public void update() {
        if (!isLearning || pendingShot == null) return;

        // Check for timeout
        if (feedbackTimer.seconds() > FEEDBACK_TIMEOUT) {
            pendingShot = null;
            return;
        }

        // Get feedback from Limelight Pipeline 0
        String feedback = getLimelightFeedback();

        if (feedback != null && !feedback.equals("pending")) {
            processFeedback(feedback);
            pendingShot = null;
        }
    }

    /**
     * Get feedback from Limelight (undershoot/overshoot/made)
     * YOU'LL NEED TO IMPLEMENT THIS BASED ON YOUR LIMELIGHT API
     */
    private String getLimelightFeedback() {
        // TODO: Read from your Limelight pipeline 0 networktables
        // Example implementation:
        /*
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double undershoot = table.getEntry("shots_undershoot").getDouble(0);
        double overshoot = table.getEntry("shots_overshoot").getDouble(0);
        double made = table.getEntry("shots_made").getDouble(0);

        if (made > 0) return "made";
        if (overshoot > 0) return "overshoot";
        if (undershoot > 0) return "undershoot";
        */

        return null; // Return null if no feedback yet
    }

    /**
     * Process the feedback and adjust power profiles
     */
    private void processFeedback(String feedback) {
        if (pendingShot == null) return;

        pendingShot.result = feedback;
        totalShots++;

        // Get the closest distance zone
        int zone = getDistanceZone(pendingShot.distance);
        PowerProfile profile = distanceProfiles.get(zone);

        if (profile == null) {
            profile = new PowerProfile(pendingShot.topPower, pendingShot.bottomPower);
            distanceProfiles.put(zone, profile);
        }

        // Add to recent shots
        profile.recentShots.add(pendingShot);
        if (profile.recentShots.size() > 10) {
            profile.recentShots.remove(0); // Keep only last 10 shots
        }

        // Adjust powers based on feedback
        double topAdjustment = 0;
        double bottomAdjustment = 0;

        switch (feedback) {
            case "made":
                successfulShots++;
                // Reinforce successful power values
                topAdjustment = 0;
                bottomAdjustment = 0;
                break;

            case "undershoot":
                // Increase both powers by learning rate
                topAdjustment = LEARNING_RATE;
                bottomAdjustment = LEARNING_RATE;
                break;

            case "overshoot":
                // Decrease both powers by learning rate
                topAdjustment = -LEARNING_RATE;
                bottomAdjustment = -LEARNING_RATE;
                break;
        }

        // Apply adjustments with exponential moving average
        double alpha = 0.3; // Weight for new data
        profile.avgTopPower = clampPower(
                profile.avgTopPower * (1 - alpha) +
                        (pendingShot.topPower + topAdjustment) * alpha
        );
        profile.avgBottomPower = clampPower(
                profile.avgBottomPower * (1 - alpha) +
                        (pendingShot.bottomPower + bottomAdjustment) * alpha
        );

        profile.sampleCount++;
    }

    /**
     * Get recommended power values for a given distance
     */
    public double[] getRecommendedPowers(double distance) {
        int zone = getDistanceZone(distance);
        PowerProfile profile = distanceProfiles.get(zone);

        if (profile == null || profile.sampleCount < REQUIRED_SAMPLES) {
            // Not enough data, use interpolation between known zones
            return interpolatePowers(distance);
        }

        return new double[] { profile.avgTopPower, profile.avgBottomPower };
    }

    /**
     * Get the distance zone (rounded to nearest 10 inches)
     */
    private int getDistanceZone(double distance) {
        return (int) (Math.round(distance / 10.0) * 10);
    }

    /**
     * Interpolate powers between known distance zones
     */
    private double[] interpolatePowers(double distance) {
        // Find closest zones
        Integer lowerZone = null;
        Integer upperZone = null;

        for (int zone : distanceProfiles.keySet()) {
            if (zone <= distance && (lowerZone == null || zone > lowerZone)) {
                lowerZone = zone;
            }
            if (zone >= distance && (upperZone == null || zone < upperZone)) {
                upperZone = zone;
            }
        }

        // If exact match or out of range, return closest
        if (lowerZone == null) {
            PowerProfile p = distanceProfiles.get(upperZone);
            return new double[] { p.avgTopPower, p.avgBottomPower };
        }
        if (upperZone == null) {
            PowerProfile p = distanceProfiles.get(lowerZone);
            return new double[] { p.avgTopPower, p.avgBottomPower };
        }
        if (lowerZone.equals(upperZone)) {
            PowerProfile p = distanceProfiles.get(lowerZone);
            return new double[] { p.avgTopPower, p.avgBottomPower };
        }

        // Linear interpolation
        PowerProfile lower = distanceProfiles.get(lowerZone);
        PowerProfile upper = distanceProfiles.get(upperZone);

        double ratio = (distance - lowerZone) / (double)(upperZone - lowerZone);

        double topPower = lower.avgTopPower + ratio * (upper.avgTopPower - lower.avgTopPower);
        double bottomPower = lower.avgBottomPower + ratio * (upper.avgBottomPower - lower.avgBottomPower);

        return new double[] { clampPower(topPower), clampPower(bottomPower) };
    }

    /**
     * Clamp power to valid range
     */
    private double clampPower(double power) {
        return Math.max(MIN_POWER, Math.min(MAX_POWER, power));
    }

    /**
     * Enable/disable learning mode
     */
    public void setLearningEnabled(boolean enabled) {
        this.isLearning = enabled;
    }

    public boolean isLearning() {
        return isLearning;
    }

    /**
     * Get accuracy percentage
     */
    public double getAccuracy() {
        if (totalShots == 0) return 0.0;
        return (successfulShots / (double) totalShots) * 100.0;
    }

    /**
     * Get total shots processed
     */
    public int getTotalShots() {
        return totalShots;
    }

    /**
     * Get successful shots
     */
    public int getSuccessfulShots() {
        return successfulShots;
    }

    /**
     * Get confidence level for a distance (0-100%)
     */
    public double getConfidence(double distance) {
        int zone = getDistanceZone(distance);
        PowerProfile profile = distanceProfiles.get(zone);

        if (profile == null) return 0.0;

        return Math.min(100.0, (profile.sampleCount / (double) REQUIRED_SAMPLES) * 100.0);
    }

    /**
     * Reset all learned data
     */
    public void resetLearning() {
        distanceProfiles.clear();
        initializeDefaultProfiles();
        totalShots = 0;
        successfulShots = 0;
        pendingShot = null;
    }

    /**
     * Get debug info for telemetry
     */
    public String getDebugInfo(double currentDistance) {
        int zone = getDistanceZone(currentDistance);
        PowerProfile profile = distanceProfiles.get(zone);

        StringBuilder sb = new StringBuilder();
        sb.append("ML Trainer Status:\n");
        sb.append("Learning: ").append(isLearning ? "ON" : "OFF").append("\n");
        sb.append("Accuracy: ").append(String.format("%.1f%%", getAccuracy())).append("\n");
        sb.append("Total Shots: ").append(totalShots).append("\n");
        sb.append("Zone: ").append(zone).append("in\n");

        if (profile != null) {
            sb.append("Samples: ").append(profile.sampleCount).append("\n");
            sb.append("Confidence: ").append(String.format("%.0f%%", getConfidence(currentDistance))).append("\n");
        }

        return sb.toString();
    }
}

/*
 * ============================================================================
 * HOW TO USE IN YOUR TELEOP (COMMENTED OUT - UNCOMMENT WHEN READY)
 * ============================================================================
 *
 * // 1. ADD TO YOUR TELEOP CLASS VARIABLES:
 * // private ShooterMLTrainer mlTrainer;
 *
 * // 2. IN init():
 * // mlTrainer = new ShooterMLTrainer(hardwareMap, IS_RED);
 * // mlTrainer.setLearningEnabled(true); // Enable learning mode
 *
 * // 3. IN loop():
 * // mlTrainer.update(); // Check for Limelight feedback
 *
 * // 4. WHEN SHOOTING (in applySystemMode() or handleShooterControls()):
 * // if (currentMode == SystemMode.SHOOT) {
 * //     double distance = limelight.getDistanceFromShoot();
 * //     double[] recommendedPowers = mlTrainer.getRecommendedPowers(distance);
 * //
 * //     // Use ML-recommended powers instead of fixed values
 * //     shooter.setManualPowers(recommendedPowers[0], recommendedPowers[1]);
 * //
 * //     // Record the shot for learning
 * //     mlTrainer.recordShotAttempt(recommendedPowers[0], recommendedPowers[1], distance);
 * // }
 *
 * // 5. ADD TO TELEMETRY (in displayTelemetry()):
 * // telemetry.addLine("╠═══ ML TRAINER ═══╣");
 * // telemetry.addData("│ Status", mlTrainer.isLearning() ? "LEARNING" : "OFF");
 * // telemetry.addData("│ Accuracy", "%.1f%%", mlTrainer.getAccuracy());
 * // telemetry.addData("│ Total Shots", mlTrainer.getTotalShots());
 * // telemetry.addData("│ Confidence", "%.0f%%", mlTrainer.getConfidence(limelight.getDistanceFromShoot()));
 *
 * ============================================================================
 */