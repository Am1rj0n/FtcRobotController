package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private final Follower follower;
    private final boolean isRed;

    private double driveSpeed = 0.7;
    private boolean isHolding = false;

    private static final double SPEED_INCREMENT = 0.1;
    private static final double MIN_SPEED = 0.1;
    private static final double MAX_SPEED = 1.0;

    // Field-centric offset
    private double fieldCentricOffset = 0.0;

    // Heading lock for SWM
    private boolean headingLockEnabled = false;
    private double lockedHeading = 0.0;
    private static final double HEADING_LOCK_P = 2.0; // From Pedro constants

    // Corner reset positions
    private static final Pose RED_CORNER = new Pose(10, 9, Math.toRadians(0));
    private static final Pose BLUE_CORNER = new Pose(135, 9, Math.toRadians(180));

    public Drivetrain(HardwareMap hardwareMap, Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed = isRed;
    }

    public void drive(double forward, double strafe, double turn) {
        // Break hold automatically if there is significant driver input
        if (isHolding && (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1)) {
            releaseHold();
        }

        // If still holding, skip manual drive
        if (isHolding) {
            return;
        }

        // Get heading from Pinpoint
        double heading = follower.getPose().getHeading() - fieldCentricOffset;

        // Field-centric conversion
        double x = strafe;
        double y = forward;

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Apply speed multiplier
        rotX *= driveSpeed;
        rotY *= driveSpeed;
        turn *= driveSpeed;

        // Heading lock override for SWM
        if (headingLockEnabled) {
            double currentHeading = follower.getPose().getHeading();
            double headingError = normalizeAngle(lockedHeading - currentHeading);
            turn = headingError * HEADING_LOCK_P;
        }

        follower.setTeleOpDrive(rotX, rotY, -turn, false);
    }

    // ========== POSITION HOLD ==========
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

    // ========== CORNER RESET ==========
    public void resetToCorner() {
        if (isRed) {
            follower.setPose(RED_CORNER);
        } else {
            follower.setPose(BLUE_CORNER);
        }
    }

    // ========== HEADING LOCK FOR SWM ==========
    public void setHeadingLock(double targetHeading) {
        headingLockEnabled = true;
        lockedHeading = targetHeading;
    }

    public void releaseHeadingLock() {
        headingLockEnabled = false;
    }

    public boolean isHeadingLocked() {
        return headingLockEnabled;
    }

    // ========== FIELD-CENTRIC ==========
    public void resetFieldCentric() {
        fieldCentricOffset = follower.getPose().getHeading();
    }

    // ========== SPEED CONTROL ==========
    public void increaseSpeed() {
        driveSpeed = Math.min(driveSpeed + SPEED_INCREMENT, MAX_SPEED);
    }

    public void decreaseSpeed() {
        driveSpeed = Math.max(driveSpeed - SPEED_INCREMENT, MIN_SPEED);
    }

    public double getSpeed() {
        return driveSpeed;
    }

    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}