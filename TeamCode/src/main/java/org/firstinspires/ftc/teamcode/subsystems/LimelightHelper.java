package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

/**
 * Helper class for Limelight 3A AprilTag detection
 * Detects DECODE 2025 obelisk patterns (Tags 21, 22, 23)
 * Uses the Limelight3A native API
 */
public class LimelightHelper {
    private Limelight3A limelight;
    private int detectedTagId = -1;

    /**
     * Initialize Limelight with AprilTag detection
     * @param hardwareMap The hardware map from the OpMode
     */
    public LimelightHelper(HardwareMap hardwareMap) {
        // Get Limelight using its native class
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to pipeline 0 for AprilTag detection
        limelight.pipelineSwitch(1);

        // Start the Limelight
        limelight.start();
    }

    /**
     * Detect AprilTag and return the ID (21, 22, or 23 for DECODE 2025 obelisk)
     * @return The detected tag ID, or -1 if no tag detected
     */
    public int detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get fiducial (AprilTag) results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                // Get the first (most confident) detection
                LLResultTypes.FiducialResult fiducial = fiducialResults.get(0);
                detectedTagId = (int) fiducial.getFiducialId();
                return detectedTagId;
            }
        }

        return -1; // No tag detected
    }

    /**
     * Get all current AprilTag detections
     * @return List of all detected AprilTags
     */
    public List<LLResultTypes.FiducialResult> getDetections() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    /**
     * Check if any AprilTag is detected
     * @return true if at least one tag is detected
     */
    public boolean hasDetection() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            return fiducials != null && !fiducials.isEmpty();
        }
        return false;
    }

    /**
     * Get the stored detected tag ID
     * @return The last detected tag ID
     */
    public int getDetectedTagId() {
        return detectedTagId;
    }

    /**
     * Get the latest result from Limelight
     * @return LLResult object with all detection data
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Close the Limelight (call when done)
     */
    public void close() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}