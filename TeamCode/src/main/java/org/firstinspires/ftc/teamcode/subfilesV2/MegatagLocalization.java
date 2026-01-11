package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class MegatagLocalization {

    private final Limelight3A limelight;

    private boolean enabled = true;
    private double lastUpdateTime = 0;

    // Update every 3 seconds to avoid fighting with Pinpoint
    private static final double UPDATE_INTERVAL = 3.0;

    // Weighted average: 30% MegaTag2, 70% Pinpoint
    private static final double MEGATAG_WEIGHT = 0.3;

    // Only use MegaTag2 when we have good data
    private static final int MIN_TAG_COUNT = 1;

    public MegatagLocalization(HardwareMap hardwareMap) {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Poll 100 times per second
        limelight.pipelineSwitch(0); // Use pipeline 0 (configure as AprilTag pipeline in web UI)
        limelight.start();
    }

    public void updatePeriodic(Follower follower, double currentTime) {
        if (!enabled) return;

        // Only update every UPDATE_INTERVAL seconds
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
            return;
        }

        // Update Limelight with current robot orientation for MegaTag2
        Pose currentPose = follower.getPose();
        double robotYawDegrees = Math.toDegrees(currentPose.getHeading());

        // Send robot orientation to Limelight for MegaTag2
        limelight.updateRobotOrientation(robotYawDegrees);

        // Get latest result
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        // Get MegaTag2 botpose
        LLResult.Pose3D botpose_mt2 = result.getBotpose_MT2();

        if (botpose_mt2 == null) {
            return;
        }

        // Check if we have enough tags
        if (result.getBotpose_MT2_tagCount() < MIN_TAG_COUNT) {
            return;
        }

        // Extract position from MegaTag2
        // NOTE: Limelight uses FTC coordinate system:
        // (0,0,0) is center of field
        // X = forward/backward, Y = left/right, Z = up/down
        double mt2_x = botpose_mt2.getPosition().x;
        double mt2_y = botpose_mt2.getPosition().y;

        // Get rotation - Limelight returns orientation
        // We need to extract yaw (rotation around Z axis)
        double mt2_yaw = Math.toRadians(botpose_mt2.getOrientation().getYaw());

        // Weighted average with current Pinpoint pose
        double corrected_x = currentPose.getX() * (1 - MEGATAG_WEIGHT) + mt2_x * MEGATAG_WEIGHT;
        double corrected_y = currentPose.getY() * (1 - MEGATAG_WEIGHT) + mt2_y * MEGATAG_WEIGHT;
        double corrected_heading = currentPose.getHeading() * (1 - MEGATAG_WEIGHT) + mt2_yaw * MEGATAG_WEIGHT;

        // Update follower pose
        Pose correctedPose = new Pose(corrected_x, corrected_y, corrected_heading);
        follower.setPose(correctedPose);

        lastUpdateTime = currentTime;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }

    public void close() {
        limelight.stop();
    }
}