package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierPoint;

public class DrivetrainSubsystem {

    private final Follower follower;
    private final boolean isRed;

    private double driveSpeed = 0.7;
    private boolean slowMode = false;
    private boolean isHolding = false;

    private static final double SPEED_INCREMENT = 0.1;
    private static final double MIN_SPEED = 0.1;
    private static final double MAX_SPEED = 1.0;
    private static final double SLOW_MODE_SPEED = 0.1;

    // Field-centric offset (for driver perspective adjustment)
    private double fieldCentricOffset = 0.0;

    // Goal tracking
    private boolean goalTrackingEnabled = false;
    private static final double GOAL_TRACK_P = 1.20; //was 0.95
    private static final double GOAL_TRACK_MAX_TURN = 0.8;
    // ADDED: Goal alignment tolerance (in radians, 0 degrees)
    private static final double GOAL_ALIGN_TOLERANCE = Math.toRadians(0);

    // Red goal at (144, 144), Blue goal at (0, 144)
    private final Pose GOAL_POSE;

    // Corner Reset Positions
    private static final Pose RED_CORNER = new Pose(10, 9, Math.toRadians(0));
    private static final Pose BLUE_CORNER = new Pose(135, 8, Math.toRadians(180));

    public DrivetrainSubsystem(HardwareMap hardwareMap, Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed = isRed;
        // Blue goal at top-left (14, 144), Red goal at top-right (130, 144)
        GOAL_POSE = isRed ? new Pose(131, 137, 0) : new Pose(13, 136, 0);
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
            turn = calculateGoalTrackingTurn();
        }

        follower.setTeleOpDrive(rotX, -rotY, -turn, false);
    }

    // ========== POSITION HOLD LOGIC ==========
    public void toggleHold() {
        if (isHolding) {
            releaseHold();
        } else {
            // Locks the robot to its current (X, Y) and Heading
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

    private double calculateGoalTrackingTurn() {
        Pose currentPose = follower.getPose();
        double targetHeading = Math.atan2(
                GOAL_POSE.getY() - currentPose.getY(),
                GOAL_POSE.getX() - currentPose.getX()
        );

        double currentHeading = currentPose.getHeading();
        double error = normalizeAngle(targetHeading - currentHeading);

        if (Math.abs(error) < 0.02) return 0;

        double turn = error * GOAL_TRACK_P;
        return -Math.max(-GOAL_TRACK_MAX_TURN, Math.min(GOAL_TRACK_MAX_TURN, turn));
    }

    // ADDED: Check if robot is aligned with goal
    public boolean isAlignedWithGoal() {
        if (!goalTrackingEnabled) return false;

        Pose currentPose = follower.getPose();
        double targetHeading = Math.atan2(
                GOAL_POSE.getY() - currentPose.getY(),
                GOAL_POSE.getX() - currentPose.getX()
        );

        double currentHeading = currentPose.getHeading();
        double error = Math.abs(normalizeAngle(targetHeading - currentHeading));

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
}