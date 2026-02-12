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

public class Limelight {

    private final Limelight3A limelight;
    private final boolean isRed;

    private int detectedTagId = -1;
    private double lastUpdateTime = 0;

    // Alignment tags
    private static final int BLUE_ALIGNMENT_TAG = 20;
    private static final int RED_ALIGNMENT_TAG = 24;

    // Localization update settings
    private static final double MIN_UPDATE_INTERVAL = 2.0; // seconds
    private static final int MIN_DETECTIONS_FOR_UPDATE = 3;
    private int consecutiveDetections = 0;

    public Limelight(HardwareMap hardwareMap, boolean isRed) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.isRed = isRed;

        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.setPollRateHz(250);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public boolean detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult fiducialResult = fiducialResults.get(0);
                detectedTagId = fiducialResult.getFiducialId();
                return true;
            }
        }

        detectedTagId = -1;
        return false;
    }

    public boolean isAlignmentTagVisible(int targetTag) {
        detectAprilTag();
        return detectedTagId == targetTag;
    }

    public double getTx() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult fiducialResult = fiducialResults.get(0);
                return fiducialResult.getTargetXDegrees();
            }
        }

        return 0.0;
    }

    public double getTy() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult fiducialResult = fiducialResults.get(0);
                return fiducialResult.getTargetYDegrees();
            }
        }

        return 0.0;
    }

    /**
     * Update robot localization using Limelight bot pose
     */
    public void updateLocalization(Follower follower, double currentTime) {
        // Rate limit updates
        if (currentTime - lastUpdateTime < MIN_UPDATE_INTERVAL) {
            return;
        }

        int targetTag = isRed ? RED_ALIGNMENT_TAG : BLUE_ALIGNMENT_TAG;

        if (!isAlignmentTagVisible(targetTag)) {
            consecutiveDetections = 0;
            return;
        }

        consecutiveDetections++;

        // Require multiple consecutive detections for confidence
        if (consecutiveDetections < MIN_DETECTIONS_FOR_UPDATE) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        Pose3D botPose = result.getBotpose();
        if (botPose == null) {
            return;
        }

        // Convert Limelight coordinates to field coordinates
        // Limelight bot pose is in meters, convert to inches
        double fieldX = 72.0 + (botPose.getPosition().y * 39.37);
        double fieldY = 72.0 - (botPose.getPosition().x * 39.37);
        double rawHeading = botPose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Normalize heading
        if (rawHeading < 0) {
            rawHeading += 360;
        }
        double fieldHeading = Math.toRadians(rawHeading - 90);

        // Update follower pose
        follower.setPose(new Pose(fieldX, fieldY, fieldHeading));

        lastUpdateTime = currentTime;
        consecutiveDetections = 0;
    }

    public boolean hasValidBotPose() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getBotpose() != null;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }
}