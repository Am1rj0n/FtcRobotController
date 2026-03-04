package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

public class Turret {

    private final Servo      turretServo;
    private final Limelight  limelight;
    private final boolean    isRed;

    public enum Mode {
        ODOMETRY,   // Track goal using odometry
        LIMELIGHT,  // Track goal using vision
        MANUAL      // Manual control (teleop sets angle directly)
    }

    private Mode   currentMode  = Mode.ODOMETRY;
    private double targetAngle  = 0.0;
    private double manualAngle  = 0.0;

    // =========================================================================
    //  TUNING — change these to adjust turret behavior
    // =========================================================================

    private static final double SERVO_CENTER = 0.5;


    private static final double MAX_ANGLE = 50.0;  // degrees — currently ±50°


    private static final double ALIGNMENT_TOLERANCE = 2.0;  // degrees

    // =========================================================================
    //  GOAL POSITIONS (inches) — must match SWM and Drivetrain
    // =========================================================================
    private static final double BLUE_GOAL_X = 7;
    private static final double BLUE_GOAL_Y = 141.0;
    private static final double RED_GOAL_X  = 134.5;
    private static final double RED_GOAL_Y  = 140.0;

    // =========================================================================
    //  CONSTRUCTOR
    // =========================================================================

    public Turret(HardwareMap hardwareMap, Limelight limelight, boolean isRed) {
        turretServo  = hardwareMap.servo.get("turret");
        this.limelight = limelight;
        this.isRed     = isRed;

        turretServo.setPosition(SERVO_CENTER);
    }

    // =========================================================================
    //  UPDATE — call every loop
    // =========================================================================

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

    // =========================================================================
    //  ANGLE COMPUTATION
    // =========================================================================


    private double calculateOdometryAngle(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        double globalAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading      = Math.toDegrees(pose.getHeading());

        double turretAngle = globalAngleToGoal - robotHeading;

        // Normalize to ±180
        while (turretAngle >  180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;

        return clampAngle(turretAngle);
    }


    private double calculateLimelightAngle() {
        if (limelight.isAlignmentTagVisible()) {
            return clampAngle(limelight.getTx());
        }
        return targetAngle; // hold last known position
    }

    // =========================================================================
    //  SERVO MAPPING
    // =========================================================================


    private void setServoAngle(double angle) {
        angle = clampAngle(angle);
        double pos = SERVO_CENTER + (angle / MAX_ANGLE) * (1.0 - SERVO_CENTER);
        pos = Math.max(0.0, Math.min(1.0, pos));
        turretServo.setPosition(pos);
    }


    private double getCurrentAngle() {
        double position = turretServo.getPosition();
        return (position - SERVO_CENTER) / (1.0 - SERVO_CENTER) * MAX_ANGLE;
    }

    // =========================================================================
    //  PUBLIC QUERIES
    // =========================================================================

    /** True if turret is within ALIGNMENT_TOLERANCE degrees of target. */
    public boolean isAligned() {
        return Math.abs(targetAngle - getCurrentAngle()) < ALIGNMENT_TOLERANCE;
    }

    /** Distance from robot to goal in meters (used by Shooter for RPM). */
    public double distanceToGoalMeters(Pose pose) {
        double goalX  = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY  = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;
        double inches = Math.hypot(goalX - pose.getX(), goalY - pose.getY());
        return inches * 0.0254;
    }

    /** Returns the current MAX_ANGLE so telemetry can display it. */
    public double getMaxAngle() { return MAX_ANGLE; }

    // =========================================================================
    //  CONTROL
    // =========================================================================

    public void setMode(Mode mode)          { currentMode = mode; }
    public void setManualAngle(double angle){ manualAngle = clampAngle(angle); }
    public void centerTurret()              { setServoAngle(0.0); }

    public Mode   getCurrentMode()  { return currentMode; }
    public double getTargetAngle()  { return targetAngle; }

    private double clampAngle(double angle) {
        return Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
    }
}