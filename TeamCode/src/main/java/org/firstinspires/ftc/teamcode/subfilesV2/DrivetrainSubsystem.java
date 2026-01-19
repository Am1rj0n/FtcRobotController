package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DrivetrainSubsystem {

    private final Follower follower;
    private final boolean isRed;
    private LimelightHelper limelightHelper;

    private double driveSpeed = 0.7;
    private boolean slowMode = false;
    private boolean isHolding = false;

    private static final double SPEED_INCREMENT = 0.1;
    private static final double MIN_SPEED = 0.1;
    private static final double MAX_SPEED = 1.0;
    private static final double SLOW_MODE_SPEED = 0.1;

    // Field-centric offset (for driver perspective adjustment)
    private double fieldCentricOffset = 0.0;

    // Goal tracking - tuned for smooth strafing alignment
    private boolean goalTrackingEnabled = false;
    private boolean visionAssistEnabled = false;  // ← NEW: Vision assist toggle
    private static final double GOAL_TRACK_P = 1.2;
    private static final double GOAL_TRACK_D = 0.0;
    private static final double GOAL_TRACK_MAX_TURN = 1.0;
    private static final double GOAL_ALIGN_TOLERANCE = Math.toRadians(1);

    // PD controller state
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Red goal at (131, 137), Blue goal at (13, 136)
    private final Pose GOAL_POSE;

    // Corner Reset Positions
    private static final Pose RED_CORNER = new Pose(10, 9, Math.toRadians(0));
    private static final Pose BLUE_CORNER = new Pose(135, 9, Math.toRadians(180));

    public DrivetrainSubsystem(HardwareMap hardwareMap, Follower follower, boolean isRed, LimelightHelper limelightHelper) {
        this.follower = follower;
        this.isRed = isRed;
        this.limelightHelper = limelightHelper;
        GOAL_POSE = isRed ? new Pose(131, 136, 0) : new Pose(13, 136, 0);
        pidTimer.reset();
    }

    public void drive(double forward, double strafe, double turn) {
        // Break hold automatically if there is significant driver input
        if (isHolding && (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1)) {
            releaseHold();
        }

        // If still holding, skip manual drive logic to let Pedro handle the hold
        if (isHolding) {
            return;
        }

        // Get heading directly from Pinpoint
        double heading = follower.getPose().getHeading() - fieldCentricOffset;

        // Field-centric conversion
        double rotX = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotY = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        // Apply speed multiplier
        double activeSpeed = slowMode ? SLOW_MODE_SPEED : driveSpeed;
        rotX *= activeSpeed;
        rotY *= activeSpeed;
        turn *= activeSpeed;

        // Goal tracking override
        if (goalTrackingEnabled && !hasSignificantTurnInput(turn)) {
            turn = calculateGoalTrackingTurnPD();
        } else {
            // Reset PD state when not tracking
            lastError = 0.0;
            pidTimer.reset();
        }

        follower.setTeleOpDrive(rotX, -rotY, -turn, false);
    }

    // ========== POSITION HOLD LOGIC ==========
    public void toggleHold() {
        if (isHolding) {
            releaseHold();
        } else {
            follower.holdPoint(new BezierPoint(follower.getPose()), follower.getPose().getHeading());
            isHolding = true;
        }
    }

    public void releaseHold() {
        isHolding = false;
        follower.startTeleopDrive();
    }

    public boolean isHolding() {
        return isHolding;
    }

    // ========== ALLIANCE CORNER RESET ==========
    public void resetToCorner() {
        if (isRed) {
            follower.setPose(RED_CORNER);
        } else {
            follower.setPose(BLUE_CORNER);
        }
    }

    // ==================== VISION ASSIST TOGGLE - NEW ====================
    /**
     * Toggle vision assistance on/off
     * Returns true if now enabled, false if now disabled
     */
    public boolean toggleVisionAssist() {
        visionAssistEnabled = !visionAssistEnabled;
        return visionAssistEnabled;
    }

    public boolean isVisionAssistEnabled() {
        return visionAssistEnabled;
    }

    // ==================== VISION-ASSISTED PD CONTROLLER - MODIFIED ====================
    /**
     * PD Controller with OPTIONAL Vision Assistance
     *
     * PRIORITY SYSTEM (only when vision assist is enabled):
     * 1. Use Limelight vision if AprilTag visible AND vision assist ON
     * 2. Fall back to odometry if no tag OR vision assist OFF
     */
    private double calculateGoalTrackingTurnPD() {
        Pose currentPose = follower.getPose();
        double currentHeading = currentPose.getHeading();

        double targetHeading;

        // Only use vision if BOTH goal tracking AND vision assist are enabled
        if (visionAssistEnabled) {
            // Try to get vision-based angle from Limelight
            double visionAngle = limelightHelper.getAngleFromShoot();

            if (Math.abs(visionAngle) > 0.5) {
                // VISION MODE - AprilTag detected and vision assist ON
                targetHeading = currentHeading + Math.toRadians(visionAngle);
            } else {
                // ODOMETRY MODE - No tag visible, use pose calculation
                targetHeading = Math.atan2(
                        GOAL_POSE.getY() - currentPose.getY(),
                        GOAL_POSE.getX() - currentPose.getX()
                );
            }
        } else {
            // Vision assist OFF - always use odometry
            targetHeading = Math.atan2(
                    GOAL_POSE.getY() - currentPose.getY(),
                    GOAL_POSE.getX() - currentPose.getX()
            );
        }

        double error = normalizeAngle(targetHeading - currentHeading);

        // Calculate derivative
        double dt = pidTimer.seconds();
        double derivative = 0.0;
        if (dt > 0.001) {
            derivative = (error - lastError) / dt;
        }

        // PD control
        double turn = (error * GOAL_TRACK_P) + (derivative * GOAL_TRACK_D);

        // Update state
        lastError = error;
        pidTimer.reset();

        // Clamp output
        turn = Math.max(-GOAL_TRACK_MAX_TURN, Math.min(GOAL_TRACK_MAX_TURN, turn));
        return -turn;
    }

    // ==================== ALIGNMENT CHECK - MODIFIED ====================
    public boolean isAlignedWithGoal() {
        if (!goalTrackingEnabled) return false;

        Pose currentPose = follower.getPose();
        double currentHeading = currentPose.getHeading();

        double error;

        if (visionAssistEnabled) {
            // Try vision first if enabled
            double visionAngle = limelightHelper.getAngleFromShoot();

            if (Math.abs(visionAngle) > 0.5) {
                // Vision-based alignment check
                error = Math.abs(Math.toRadians(visionAngle));
            } else {
                // Odometry-based alignment check
                double targetHeading = Math.atan2(
                        GOAL_POSE.getY() - currentPose.getY(),
                        GOAL_POSE.getX() - currentPose.getX()
                );
                error = Math.abs(normalizeAngle(targetHeading - currentHeading));
            }
        } else {
            // Vision assist OFF - use odometry only
            double targetHeading = Math.atan2(
                    GOAL_POSE.getY() - currentPose.getY(),
                    GOAL_POSE.getX() - currentPose.getX()
            );
            error = Math.abs(normalizeAngle(targetHeading - currentHeading));
        }

        return error < GOAL_ALIGN_TOLERANCE;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private boolean hasSignificantTurnInput(double turn) {
        return Math.abs(turn) > 0.05;
    }

    public void resetFieldCentric() {
        fieldCentricOffset = follower.getPose().getHeading();
    }

    public void setSlowMode(boolean enabled) {
        slowMode = enabled;
    }

    public void increaseSpeed() {
        driveSpeed = Math.min(driveSpeed + SPEED_INCREMENT, MAX_SPEED);
    }

    public void decreaseSpeed() {
        driveSpeed = Math.max(driveSpeed - SPEED_INCREMENT, MIN_SPEED);
    }

    public void toggleGoalTracking() {
        goalTrackingEnabled = !goalTrackingEnabled;
        if (goalTrackingEnabled) {
            lastError = 0.0;
            pidTimer.reset();
        }
    }

    public boolean isGoalTrackingEnabled() {
        return goalTrackingEnabled;
    }

    public double getSpeed() {
        return driveSpeed;
    }

    public boolean hasManualInput(Gamepad gamepad) {
        return Math.abs(gamepad.left_stick_x) > 0.1 ||
                Math.abs(gamepad.left_stick_y) > 0.1 ||
                Math.abs(gamepad.right_stick_x) > 0.1;
    }

    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    // ==================== HELPER METHOD FOR DEBUGGING ====================
    public boolean isUsingVision() {
        return visionAssistEnabled && Math.abs(limelightHelper.getAngleFromShoot()) > 0.5;
    }
}