package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

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

    private static final double SERVO_CENTER = 0.5;
    private static final double MIN_ANGLE    = -24.0; // degrees
    private static final double MAX_ANGLE    =  24.0; // degrees

    private static final double ALIGNMENT_TOLERANCE = 2.0; // degrees

    // Goal positions (inches)
    private static final double BLUE_GOAL_X = 0.0;
    private static final double BLUE_GOAL_Y = 144.0;
    private static final double RED_GOAL_X  = 144.0;
    private static final double RED_GOAL_Y  = 144.0;

    public Turret(HardwareMap hardwareMap, Limelight limelight, boolean isRed) {
        turretServo = hardwareMap.servo.get("turret");
        this.limelight = limelight;
        this.isRed = isRed;

        turretServo.setPosition(SERVO_CENTER);
    }

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
        return targetAngle; // Hold last position if no tag
    }


    private void setServoAngle(double angle) {
        angle = clampAngle(angle);
        double servoPosition = SERVO_CENTER + (angle / MAX_ANGLE) * (1.0 - SERVO_CENTER);
        turretServo.setPosition(servoPosition);
    }


    private double getCurrentAngle() {
        double position = turretServo.getPosition();
        return (position - SERVO_CENTER) / (1.0 - SERVO_CENTER) * MAX_ANGLE;
    }


    public boolean isAligned() {
        return Math.abs(targetAngle - getCurrentAngle()) < ALIGNMENT_TOLERANCE;
    }


    public double distanceToGoalMeters(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double inches = Math.hypot(goalX - pose.getX(), goalY - pose.getY());
        return inches * 0.0254;
    }

    // ==================== CONTROL ====================

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void setManualAngle(double angle) {
        manualAngle = clampAngle(angle);
    }

    public Mode getCurrentMode()  { return currentMode; }
    public double getTargetAngle(){ return targetAngle; }

    private double clampAngle(double angle) {
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }
}