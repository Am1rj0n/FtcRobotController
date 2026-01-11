package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class DrivetrainSubsystem {

    private final Follower follower;
    private final GoBildaPinpointDriver pinpoint;
    private final boolean isRed;

    private double driveSpeed = 1.0;
    private boolean slowMode = false;
    private static final double SPEED_INCREMENT = 0.1;
    private static final double MIN_SPEED = 0.1;
    private static final double MAX_SPEED = 1.0;
    private static final double SLOW_MODE_SPEED = 0.1;

    // Field-centric offset (for driver perspective adjustment)
    private double fieldCentricOffset = 0.0;

    // Goal tracking
    private boolean goalTrackingEnabled = false;
    private static final double GOAL_TRACK_P = 2.2;
    private static final double GOAL_TRACK_MAX_TURN = 0.6;

    // Red goal at (144, 144), Blue goal at (0, 144)
    private final Pose GOAL_POSE;

    public DrivetrainSubsystem(HardwareMap hardwareMap, Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed = isRed;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Initialize Pinpoint
        pinpoint.setOffsets(-84.0, -168.0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();

        // Set goal pose based on alliance
        GOAL_POSE = isRed ? new Pose(144, 144, 0) : new Pose(0, 144, 0);
    }

    public void drive(double forward, double strafe, double turn) {
        // Get heading from Pinpoint IMU and apply field-centric offset
        double heading = pinpoint.getHeading() - fieldCentricOffset;

        // Field-centric conversion
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);

        // Apply speed multiplier (slow mode overrides normal speed)
        double activeSpeed = slowMode ? SLOW_MODE_SPEED : driveSpeed;
        rotX *= activeSpeed;
        rotY *= activeSpeed;
        turn *= activeSpeed;

        // Goal tracking override
        if (goalTrackingEnabled && !hasSignificantTurnInput(turn)) {
            turn = calculateGoalTrackingTurn();
        }

        // Send to follower
        follower.setTeleOpDrive(rotY, rotX, turn, false, 0);
    }

    private double calculateGoalTrackingTurn() {
        Pose currentPose = follower.getPose();
        double targetHeading = Math.atan2(
                GOAL_POSE.getY() - currentPose.getY(),
                GOAL_POSE.getX() - currentPose.getX()
        );

        double currentHeading = currentPose.getHeading();
        double error = normalizeAngle(targetHeading - currentHeading);

        double turn = error * GOAL_TRACK_P;
        return Math.max(-GOAL_TRACK_MAX_TURN, Math.min(GOAL_TRACK_MAX_TURN, turn));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private boolean hasSignificantTurnInput(double turn) {
        return Math.abs(turn) > 0.05;
    }

    public void resetIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void resetFieldCentric() {
        // Store current heading as offset - this makes field-centric feel like it's from driver's POV
        fieldCentricOffset = pinpoint.getHeading();
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
        follower.setTeleOpDrive(0, 0, 0, true, 0);
    }
}