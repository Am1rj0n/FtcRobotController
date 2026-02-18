package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight - Pure Odometry + MegaTag2 on demand
 *
 * Fusion localization has been removed.
 * Pedro Pinpoint handles all continuous localization.
 * MegaTag2 is used only when triggered by button press to correct drift.
 */
public class Limelight {

    private final Limelight3A limelight;
    private final boolean isRed;

    private int detectedTagId = -1;

    // Alignment tags for turret tracking
    private static final int BLUE_ALIGNMENT_TAG = 20;
    private static final int RED_ALIGNMENT_TAG = 24;

    public Limelight(HardwareMap hardwareMap, boolean isRed) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.isRed = isRed;

        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.setPollRateHz(100);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    // ==================== TURRET HELPERS ====================

    /**
     * Detect AprilTag - updates detectedTagId
     */
    public boolean detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                detectedTagId = fiducialResults.get(0).getFiducialId();
                return true;
            }
        }

        detectedTagId = -1;
        return false;
    }

    /**
     * Check if alignment tag for this alliance is visible
     */
    public boolean isAlignmentTagVisible() {
        detectAprilTag();
        int targetTag = isRed ? RED_ALIGNMENT_TAG : BLUE_ALIGNMENT_TAG;
        return detectedTagId == targetTag;
    }

    /**
     * Horizontal offset to target (for turret Limelight mode)
     */
    public double getTx() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                return fiducialResults.get(0).getTargetXDegrees();
            }
        }
        return 0.0;
    }

    /**
     * Vertical offset to target
     */
    public double getTy() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                return fiducialResults.get(0).getTargetYDegrees();
            }
        }
        return 0.0;
    }

    // ==================== MEGATAG2 ON-DEMAND LOCALIZATION ====================

    /**
     * MEGATAG2 LOCALIZATION - Call this on button press only.
     *
     * Fully overrides Pedro's pose with vision estimate.
     * Much more accurate than MegaTag1 - eliminates ambiguity using robot heading.
     *
     * REQUIRES: updateMegaTag2Orientation() called every loop BEFORE this.
     *
     * @return true if localization succeeded, false if no tags visible
     */
    public boolean megaTag2Localize(Follower follower) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        Pose3D botpose_mt2 = result.getBotpose_MT2();
        if (botpose_mt2 == null) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) {
            return false;
        }

        // Convert MegaTag2 pose (meters) to Pedro field coordinates (inches)
        double visionX = botpose_mt2.getPosition().x * 39.37;
        double visionY = botpose_mt2.getPosition().y * 39.37;

        double rawHeading = botpose_mt2.getOrientation().getYaw(AngleUnit.DEGREES);
        if (rawHeading < 0) rawHeading += 360;
        double visionHeading = Math.toRadians(rawHeading);

        // Full pose override - trust MegaTag2 completely
        follower.setPose(new Pose(visionX, visionY, visionHeading));

        return true;
    }

    /**
     * Update robot orientation for MegaTag2.
     * MUST be called every loop for MegaTag2 to work correctly.
     * Sends current robot yaw from Pedro/Pinpoint to the Limelight.
     */
    public void updateMegaTag2Orientation(Follower follower) {
        double robotYaw = Math.toDegrees(follower.getHeading());
        limelight.updateRobotOrientation(robotYaw);
    }

    // ==================== TELEMETRY HELPERS ====================

    public int getVisibleTagCount() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;
        return result.getFiducialResults().size();
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }
}