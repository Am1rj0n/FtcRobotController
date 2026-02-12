package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret {

    private final Servo turretServo;
    private final Limelight limelight;
    private final boolean isRed;

    public enum Mode {
        LIMELIGHT,    // Pure vision (L2)
        ODOMETRY,     // Odometry + vision assist (R2)
        MANUAL        // Manual control
    }

    private Mode currentMode = Mode.MANUAL;
    private double targetAngle = 0.0;
    private double manualAngle = 0.0;

    // Turret limits: ±58° (116° total range)
    private static final double MIN_ANGLE = -58.0;
    private static final double MAX_ANGLE = 58.0;
    private static final double ALIGNMENT_TOLERANCE = 2.0; // degrees

    // Vision correction gain (for odometry mode)
    private static final double VISION_CORRECTION_GAIN = 0.5;

    // Goal positions (inches)
    private static final double BLUE_GOAL_X = 72.0;
    private static final double BLUE_GOAL_Y = 72.0;
    private static final double RED_GOAL_X = 72.0;
    private static final double RED_GOAL_Y = -72.0;

    public Turret(HardwareMap hardwareMap, Limelight limelight, boolean isRed) {
        turretServo = hardwareMap.servo.get("turret");
        this.limelight = limelight;
        this.isRed = isRed;

        // Initialize to center position
        setServoAngle(0);
    }

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void update(Pose robotPose) {
        switch (currentMode) {
            case LIMELIGHT:
                updateLimelightMode();
                break;

            case ODOMETRY:
                updateOdometryMode(robotPose);
                break;

            case MANUAL:
                targetAngle = manualAngle;
                break;
        }

        setServoAngle(targetAngle);
    }

    /**
     * Pure Limelight alignment - uses tx directly
     */
    private void updateLimelightMode() {
        int targetTag = isRed ? 24 : 20;

        if (limelight.isAlignmentTagVisible(targetTag)) {
            double tx = limelight.getTx();
            // tx is already the angle offset we need
            targetAngle = clampAngle(tx);
        }
        // If tag lost, hold last position
    }

    /**
     * Odometry-based with vision correction when tag visible
     */
    private void updateOdometryMode(Pose pose) {
        // Calculate base angle to goal from odometry
        double heading = pose.getHeading();
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        // Transform to robot-local coordinates
        double localDx = (dx * Math.cos(heading)) + (dy * Math.sin(heading));
        double localDy = -(dx * Math.sin(heading)) + (dy * Math.cos(heading));

        double baseAngle = Math.toDegrees(Math.atan2(localDy, localDx));

        // Apply vision correction if tag visible
        int targetTag = isRed ? 24 : 20;
        if (limelight.isAlignmentTagVisible(targetTag)) {
            double txOffset = limelight.getTx();
            baseAngle += (txOffset * VISION_CORRECTION_GAIN);
        }

        targetAngle = clampAngle(baseAngle);
    }

    public void setManualAngle(double angle) {
        manualAngle = clampAngle(angle);
    }

    private void setServoAngle(double angle) {
        double servoPosition = interpolateAngle(angle);
        turretServo.setPosition(servoPosition);
    }

    /**
     * Map angle [-58, 58] to servo position [0, 1]
     */
    private double interpolateAngle(double angle) {
        angle = clampAngle(angle);
        return (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
    }

    private double clampAngle(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public boolean isAligned() {
        // Check if current angle is close to target
        return Math.abs(targetAngle - getCurrentAngle()) < ALIGNMENT_TOLERANCE;
    }

    private double getCurrentAngle() {
        // Back-calculate from servo position
        double position = turretServo.getPosition();
        return MIN_ANGLE + (position * (MAX_ANGLE - MIN_ANGLE));
    }

    public Mode getCurrentMode() {
        return currentMode;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double distanceToGoal(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        return Math.hypot(dx, dy);
    }
}