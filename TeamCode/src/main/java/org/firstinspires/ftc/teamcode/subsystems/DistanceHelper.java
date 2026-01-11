package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class DistanceHelper {

    private FilteredLimelightHelper limelight;

    /**
     * Constructor that accepts existing LimelightHelper instance
     * This prevents creating multiple Limelight instances
     */
    public DistanceHelper(FilteredLimelightHelper limelight) {
        this.limelight = limelight;
    }

    /**
     * Returns estimated forward distance to AprilTag in meters.
     * Returns null if no tag is visible.
     */
    // In DistanceHelper.java, update getDistanceMeters():

    public Double getDistanceMeters() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        LLResultTypes.FiducialResult tag = tags.get(0);
        Pose3D pose = tag.getTargetPoseRobotSpace();
        if (pose == null) return null;

        // NORMAL ORIENTATION:
        return pose.getPosition().z;  // Z-axis is forward distance
    }
}