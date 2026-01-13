package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class LimelightHelper {

    private final Limelight3A limelight;
    private final boolean isRed;
    private double lastUpdateTime = 0;

    // AprilTag IDs
    private static final int RED_SHOOT_TAG = 24;
    private static final int BLUE_SHOOT_TAG = 20;

    // Update localization every 3 seconds
    private static final double UPDATE_INTERVAL = 3.0;
    private static final double LOCALIZATION_WEIGHT = 0.3;

    public LimelightHelper(HardwareMap hardwareMap, boolean isRed) {
        this.isRed = isRed;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(0); // Pipeline 0 for AprilTags
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    // Get distance from shooting tag (in inches)
    public double getDistanceFromShoot() {
        int targetTag = isRed ? RED_SHOOT_TAG : BLUE_SHOOT_TAG;
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return 0;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null && fiducial.getFiducialId() == targetTag) {
                // Get camera pose in target space (most useful)
                Pose3D cameraPose = fiducial.getCameraPoseTargetSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x / DistanceUnit.mPerInch;
                    double z = cameraPose.getPosition().z / DistanceUnit.mPerInch;
                    return Math.sqrt(x*x + z*z);
                }
            }
        }

        return 0;
    }

    // Get angle from shooting tag (in degrees)
    public double getAngleFromShoot() {
        int targetTag = isRed ? RED_SHOOT_TAG : BLUE_SHOOT_TAG;
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return 0;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null && fiducial.getFiducialId() == targetTag) {
                return fiducial.getTargetXDegrees();
            }
        }

        return 0;
    }

    // Update robot pose using Limelight (like Barron's code)
    public void updateLocalization(Follower follower, double currentTime) {
        // Only update every UPDATE_INTERVAL seconds
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        // Get botpose from Limelight (MegaTag 1)
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return;
        }

        // Check if we have enough tags for reliable data
        int tagCount = result.getBotposeTagCount();
        if (tagCount < 1) {
            return;
        }

        // Extract X, Y from botpose
        double ll_x = botpose.getPosition().x;
        double ll_y = botpose.getPosition().y;

        // Get current follower pose
        Pose currentPose = follower.getPose();

        // Weighted average (30% Limelight, 70% Pinpoint Odometry)
        double corrected_x = currentPose.getX() * (1 - LOCALIZATION_WEIGHT) + ll_x * LOCALIZATION_WEIGHT;
        double corrected_y = currentPose.getY() * (1 - LOCALIZATION_WEIGHT) + ll_y * LOCALIZATION_WEIGHT;

        // Update follower pose (keep current heading - don't trust Limelight heading)
        Pose correctedPose = new Pose(corrected_x, corrected_y, currentPose.getHeading());
        follower.setPose(correctedPose);

        lastUpdateTime = currentTime;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }
}