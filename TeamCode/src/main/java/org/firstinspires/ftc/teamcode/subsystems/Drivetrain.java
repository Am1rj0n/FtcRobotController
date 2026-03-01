package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private final Follower follower;
    private final boolean isRed;

    private double driveSpeed = 1.0;
    private boolean isHolding = false;

    private static final double SPEED_INCREMENT = 0.1;
    private static final double MIN_SPEED       = 0.1;
    private static final double MAX_SPEED       = 1.0;

    // Field-centric offset
    private double fieldCentricOffset = 0.0;

    // Heading lock for SWM
    private boolean headingLockEnabled = false;
    private double  lockedHeading      = 0.0;
    private static final double HEADING_LOCK_P = 2.0;

    // Goal tracking (drivetrain alignment - for no-turret mode)
    private boolean goalTrackingEnabled = false;
    private boolean visionAssistEnabled = false;
    private static final double GOAL_TRACK_P        = 1.2;
    private static final double GOAL_TRACK_D        = 0.05;
    private static final double GOAL_TRACK_MAX_TURN = 1.0;
    public  static final double GOAL_ALIGN_TOLERANCE = Math.toRadians(1.5);

    private double      lastError = 0.0;
    private ElapsedTime pidTimer  = new ElapsedTime();

    // Goal positions
    private final Pose GOAL_POSE;
    private static final double BLUE_GOAL_X = 7;
    private static final double BLUE_GOAL_Y = 141;
    private static final double RED_GOAL_X  = 134.5; //was 144
    private static final double RED_GOAL_Y  = 140; //was 130
    // Corner reset positions
    private static final Pose RED_CORNER  = new Pose(119,  130, Math.toRadians(37)); //WAS 10,0
    private static final Pose BLUE_CORNER = new Pose(27, 129, Math.toRadians(143)); //was 135,9

    // Limelight reference for vision-assisted goal tracking
    private Limelight limelight = null;

    public Drivetrain(HardwareMap hardwareMap, Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed    = isRed;
        fieldCentricOffset = follower.getPose().getHeading();
        GOAL_POSE = isRed
                ? new Pose(RED_GOAL_X,  RED_GOAL_Y,  0)
                : new Pose(BLUE_GOAL_X, BLUE_GOAL_Y, 0);
    }

    public void setLimelight(Limelight limelight) {
        this.limelight = limelight;
    }

    /**
     * Field-centric drive using Pedro's odometry heading.
     * Used when drivetrain.drive() is called directly (not GM0 mode).
     */
    public void drive(double forward, double strafe, double turn) {
        if (isHolding && (Math.abs(forward) > 0.1 || Math.abs(strafe) > 0.1 || Math.abs(turn) > 0.1)) {
            releaseHold();
        }
        if (isHolding) return;

        double botHeading = follower.getPose().getHeading() - fieldCentricOffset;

        double x    = strafe;
        double y    = forward;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        rotX *= driveSpeed;
        rotY *= driveSpeed;
        turn *= driveSpeed;

        if (headingLockEnabled) {
            double currentHeading = follower.getPose().getHeading();
            double headingError   = normalizeAngle(lockedHeading - currentHeading);
            turn = headingError * HEADING_LOCK_P;
        } else if (goalTrackingEnabled && !hasSignificantTurnInput(turn / driveSpeed)) {
            turn = calculateGoalTrackingTurnPD();
        } else if (goalTrackingEnabled) {
            lastError = 0.0;
            pidTimer.reset();
        }

        follower.setTeleOpDrive(rotX, rotY, -turn, false);
    }

    // ========== GOAL TRACKING PD ==========

    private double calculateGoalTrackingTurnPD() {
        Pose   currentPose    = follower.getPose();
        double currentHeading = currentPose.getHeading();
        double targetHeading;

        if (visionAssistEnabled && limelight != null) {
            double tx = limelight.getTx();
            if (Math.abs(tx) > 0.5) {
                targetHeading = currentHeading + Math.toRadians(tx);
            } else {
                targetHeading = Math.atan2(
                        GOAL_POSE.getY() - currentPose.getY(),
                        GOAL_POSE.getX() - currentPose.getX()
                );
            }
        } else {
            targetHeading = Math.atan2(
                    GOAL_POSE.getY() - currentPose.getY(),
                    GOAL_POSE.getX() - currentPose.getX()
            );
        }

        double error      = normalizeAngle(targetHeading - currentHeading);
        double dt         = pidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - lastError) / dt : 0.0;
        double turn       = (error * GOAL_TRACK_P) + (derivative * GOAL_TRACK_D);

        turn      = Math.max(-GOAL_TRACK_MAX_TURN, Math.min(GOAL_TRACK_MAX_TURN, turn));
        lastError = error;
        pidTimer.reset();

        return -turn;
    }

    /**
     * Public getter so GM0-style TeleOps can inject the PD correction
     * directly into their own motor math without calling drive().
     */
    public double getGoalTrackingTurn() {
        return calculateGoalTrackingTurnPD();
    }

    public boolean isAlignedWithGoal() {
        if (!goalTrackingEnabled) return false;

        Pose   currentPose    = follower.getPose();
        double currentHeading = currentPose.getHeading();
        double error;

        if (visionAssistEnabled && limelight != null) {
            double tx = limelight.getTx();
            if (Math.abs(tx) > 0.5) {
                error = Math.abs(Math.toRadians(tx));
            } else {
                double targetHeading = Math.atan2(
                        GOAL_POSE.getY() - currentPose.getY(),
                        GOAL_POSE.getX() - currentPose.getX()
                );
                error = Math.abs(normalizeAngle(targetHeading - currentHeading));
            }
        } else {
            double targetHeading = Math.atan2(
                    GOAL_POSE.getY() - currentPose.getY(),
                    GOAL_POSE.getX() - currentPose.getX()
            );
            error = Math.abs(normalizeAngle(targetHeading - currentHeading));
        }

        return error < GOAL_ALIGN_TOLERANCE;
    }

    // ========== GOAL TRACKING TOGGLES ==========

    public void enableGoalTracking(boolean vision) {
        goalTrackingEnabled = true;
        visionAssistEnabled = vision;
        lastError = 0.0;
        pidTimer.reset();
    }

    public void disableGoalTracking() {
        goalTrackingEnabled = false;
        visionAssistEnabled = false;
        lastError = 0.0;
    }

    public boolean isGoalTrackingEnabled() { return goalTrackingEnabled; }
    public boolean isVisionAssistEnabled() { return visionAssistEnabled; }

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

    public boolean isHolding() { return isHolding; }

    // ========== CORNER RESET ==========
    public void resetToCorner() {
        follower.setPose(isRed ? RED_CORNER : BLUE_CORNER);
    }

    // ========== HEADING LOCK FOR SWM ==========
    public void setHeadingLock(double targetHeading) {
        headingLockEnabled = true;
        lockedHeading      = targetHeading;
    }

    public void releaseHeadingLock() {
        headingLockEnabled = false;
    }

    public boolean isHeadingLocked() { return headingLockEnabled; }

    // ========== FIELD-CENTRIC ==========
    public void resetFieldCentric() {
        fieldCentricOffset = follower.getPose().getHeading();
    }

    // ========== SPEED ==========
    public void increaseSpeed() { driveSpeed = Math.min(driveSpeed + SPEED_INCREMENT, MAX_SPEED); }
    public void decreaseSpeed() { driveSpeed = Math.max(driveSpeed - SPEED_INCREMENT, MIN_SPEED); }
    public double getSpeed()    { return driveSpeed; }

    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private boolean hasSignificantTurnInput(double rawTurn) {
        return Math.abs(rawTurn) > 0.05;
    }
}