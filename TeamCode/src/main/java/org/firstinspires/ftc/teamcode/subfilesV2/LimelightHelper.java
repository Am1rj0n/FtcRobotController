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

    // Localization tuning
    private static final double UPDATE_INTERVAL = 1.0; // seconds
    private static final double LOCALIZATION_WEIGHT = 0.3;
    private static final double MAX_JUMP_INCHES = 12.0;

    public LimelightHelper(HardwareMap hardwareMap, boolean isRed) {
        this.isRed = isRed;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);
        limelight.pipelineSwitch(1); // AprilTag pipeline
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    // ===================== SHOOTING TAG =====================

    public double getDistanceFromShoot() {
        int targetTag = isRed ? RED_SHOOT_TAG : BLUE_SHOOT_TAG;
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return 0;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) return 0;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial != null && fiducial.getFiducialId() == targetTag) {
                Pose3D cameraPose = fiducial.getCameraPoseTargetSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x / DistanceUnit.mPerInch;
                    double z = cameraPose.getPosition().z / DistanceUnit.mPerInch;
                    return Math.sqrt(x * x + z * z);
                }
            }
        }
        return 0;
    }

    public double getAngleFromShoot() {
        int targetTag = isRed ? RED_SHOOT_TAG : BLUE_SHOOT_TAG;
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return 0;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial != null && fiducial.getFiducialId() == targetTag) {
                return fiducial.getTargetXDegrees();
            }
        }
        return 0;
    }

    // ===================== LOCALIZATION =====================

    public void updateLocalization(Follower follower, double currentTime) {

        if (currentTime - lastUpdateTime < UPDATE_INTERVAL) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        // FTC SDK uses getBotpose() for MegaTag 1
        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        int tagCount = result.getBotposeTagCount();
        if (tagCount < 1) return;

        Pose currentPose = follower.getPose();

        // Convert meters → inches
        double ll_x = botpose.getPosition().x / DistanceUnit.mPerInch;
        double ll_y = botpose.getPosition().y / DistanceUnit.mPerInch;

        // Reject large jumps
        if (Math.hypot(ll_x - currentPose.getX(),
                ll_y - currentPose.getY()) > MAX_JUMP_INCHES) {
            return;
        }

        double corrected_x =
                currentPose.getX() * (1 - LOCALIZATION_WEIGHT) + ll_x * LOCALIZATION_WEIGHT;
        double corrected_y =
                currentPose.getY() * (1 - LOCALIZATION_WEIGHT) + ll_y * LOCALIZATION_WEIGHT;

        follower.setPose(new Pose(corrected_x, corrected_y, currentPose.getHeading()));
        lastUpdateTime = currentTime;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }
}
