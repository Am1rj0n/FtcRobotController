package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

/**
 * Turret with Odometry and Limelight tracking
 * Servo at 0.5 = turret aligned with robot heading
 */
public class Turret {

    private final Servo turretServo;
    private final Limelight limelight;
    private final boolean isRed;

    public enum Mode {
        ODOMETRY,   // Track goal using odometry
        LIMELIGHT,  // Track goal using vision
        MANUAL      // Manual control
    }

    private Mode currentMode = Mode.ODOMETRY;
    private double targetAngle = 0.0;
    private double manualAngle = 0.0;

    // Servo range: 0.5 ± range gives symmetric motion
    private static final double SERVO_CENTER = 0.5;
    private static final double MIN_ANGLE = -58.0;  // degrees
    private static final double MAX_ANGLE = 58.0;   // degrees

    // How close to consider aligned
    private static final double ALIGNMENT_TOLERANCE = 3.0; // degrees

    // Goal positions
    private static final double BLUE_GOAL_X = 0.0;
    private static final double BLUE_GOAL_Y = 144.0;
    private static final double RED_GOAL_X = 144.0;
    private static final double RED_GOAL_Y = 144.0;

    public Turret(HardwareMap hardwareMap, Limelight limelight, boolean isRed) {
        turretServo = hardwareMap.servo.get("turret");
        this.limelight = limelight;
        this.isRed = isRed;

        // Initialize at center (aligned with robot)
        turretServo.setPosition(SERVO_CENTER);
    }

    /**
     * Main update - call every loop
     */
    public void update(Pose robotPose) {
        switch (currentMode) {
            case ODOMETRY:
                targetAngle = calculateOdometryAngle(robotPose);
                break;

            case LIMELIGHT:
                targetAngle = calculateLimelightAngle();
                break;

            case MANUAL:
                targetAngle = manualAngle;
                break;
        }

        setServoAngle(targetAngle);
    }

    /**
     * Odometry mode: Calculate angle to goal from current robot pose
     */
    private double calculateOdometryAngle(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        // Angle to goal in global frame
        double globalAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading
        double robotHeading = Math.toDegrees(pose.getHeading());

        // Turret angle is relative to robot
        // When robot faces goal, turret should be at 0°
        double turretAngle = globalAngleToGoal - robotHeading;

        // Normalize to ±180
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return clampAngle(turretAngle);
    }

    /**
     * Limelight mode: Use vision offset
     */
    private double calculateLimelightAngle() {
        if (limelight.isAlignmentTagVisible()) {
            // Limelight gives direct offset angle
            return clampAngle(limelight.getTx());
        }
        // If no tag, hold current position
        return targetAngle;
    }

    /**
     * Convert angle to servo position
     * 0.5 = center (0°), 0.0 = -58°, 1.0 = +58°
     */
    private void setServoAngle(double angle) {
        angle = clampAngle(angle);
        double servoPosition = SERVO_CENTER + (angle / (MAX_ANGLE - MIN_ANGLE));
        turretServo.setPosition(servoPosition);
    }

    /**
     * Clamp angle to physical limits
     */
    private double clampAngle(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    /**
     * Check if turret is aligned with target
     */
    public boolean isAligned() {
        double currentAngle = getCurrentAngle();
        return Math.abs(targetAngle - currentAngle) < ALIGNMENT_TOLERANCE;
    }

    /**
     * Get current turret angle from servo position
     */
    private double getCurrentAngle() {
        double position = turretServo.getPosition();
        return (position - SERVO_CENTER) * (MAX_ANGLE - MIN_ANGLE);
    }

    /**
     * Distance to goal in meters (for auto-RPM)
     */
    public double distanceToGoalMeters(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        double inches = Math.hypot(dx, dy);
        return inches * 0.0254; // inches to meters
    }

    // ==================== CONTROL ====================

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void setManualAngle(double angle) {
        manualAngle = clampAngle(angle);
    }

    public Mode getCurrentMode() {
        return currentMode;
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}