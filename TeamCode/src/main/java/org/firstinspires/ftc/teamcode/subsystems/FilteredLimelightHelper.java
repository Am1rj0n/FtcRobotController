package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class FilteredLimelightHelper {
    private Limelight3A limelight;
    private int detectedTagId = -1;

    private static final int[] ALIGNMENT_TAGS = {20, 24};
    private static final int[] ALL_OBELISK_TAGS = {20, 21, 22, 23, 24};

    public FilteredLimelightHelper(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public int detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {

                LLResultTypes.FiducialResult fiducial = fiducialResults.get(0);
                detectedTagId = (int) fiducial.getFiducialId();
                return detectedTagId;
            }
        }

        return -1;
    }
    public boolean isAlignmentTag() {
        for (int tagId : ALIGNMENT_TAGS) {
            if (detectedTagId == tagId) {
                return true;
            }
        }
        return false;
    }
    public boolean isObeliskTag() {
        for (int tagId : ALL_OBELISK_TAGS) {
            if (detectedTagId == tagId) {
                return true;
            }
        }
        return false;
    }
    public LLResult getLatestAlignmentResult() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                int tagId = (int) fiducial.getFiducialId();

                // Check if this is an alignment tag
                for (int alignTag : ALIGNMENT_TAGS) {
                    if (tagId == alignTag) {
                        detectedTagId = tagId;
                        return result;
                    }
                }
            }
        }

        return null;
    }
    public List<LLResultTypes.FiducialResult> getDetections() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }
    public boolean hasDetection() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            return fiducials != null && !fiducials.isEmpty();
        }
        return false;
    }
    public int getDetectedTagId() {
        return detectedTagId;
    }
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
    public void close() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}