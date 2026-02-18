package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Point;

/**
 * Shooting While Moving with Marrow Zones
 * - Fixed flight time (ONE constant to tune)
 * - Uses Pedro's getVelocity() and getPose()
 * - Auto-shoots when in designated zones
 */
public class ShootingWhileMoving {

    private final Follower follower;
    private final Shooter shooter;
    private final Turret turret;
    private final boolean isRed;

    // ==================== SINGLE TUNING CONSTANT ====================
    // Tune based on average shot distance
    // Start at 0.5s, increase if short, decrease if long
    private static final double FLIGHT_TIME_SECONDS = 0.5;

    // ==================== MARROW SHOOT ZONES ====================
    // These define WHERE on the field auto-shoot can happen
    private final PolygonZone closeLaunchZone = new PolygonZone(
            new Point(144, 144),
            new Point(72, 72),
            new Point(0, 144)
    );

    private final PolygonZone farLaunchZone = new PolygonZone(
            new Point(48, 0),
            new Point(72, 24),
            new Point(96, 0)
    );

    // Robot footprint for zone collision detection
    private final PolygonZone robotZone = new PolygonZone(14, 18);

    private static final double BLUE_GOAL_X = 0.0;
    private static final double BLUE_GOAL_Y = 144.0;
    private static final double RED_GOAL_X = 144.0;
    private static final double RED_GOAL_Y = 144.0;

    private boolean enabled = false;
    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    public ShootingWhileMoving(Follower follower, Shooter shooter, Turret turret, boolean isRed) {
        this.follower = follower;
        this.shooter = shooter;
        this.turret = turret;
        this.isRed = isRed;
    }

    public void toggle() {
        enabled = !enabled;

        if (!enabled) {
            headingLockActive = false;
        }
    }

    /**
     * Update - checks if robot is in shoot zones
     * Call this every loop
     */
    public void update() {
        if (!enabled) {
            headingLockActive = false;
            return;
        }

        Pose currentPose = follower.getPose();

        // Update robot zone position for collision detection
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        // Check if robot is in either shoot zone
        boolean inShootZone = robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);

        if (inShootZone) {
            // Calculate heading to goal from future position
            Pose futurePose = getFuturePose();
            double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
            double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

            targetHeading = Math.atan2(goalY - futurePose.getY(), goalX - futurePose.getX());
            headingLockActive = true;
        } else {
            headingLockActive = false;
        }
    }

    /**
     * Future pose using Pedro's velocity
     */
    public Pose getFuturePose() {
        Pose current = follower.getPose();
        Vector vel = follower.getVelocity();

        return new Pose(
                current.getX() + (vel.getXComponent() * FLIGHT_TIME_SECONDS),
                current.getY() + (vel.getYComponent() * FLIGHT_TIME_SECONDS),
                current.getHeading()
        );
    }

    /**
     * Heading to shoot from future position
     */
    public double getTargetHeading() {
        return targetHeading;
    }

    /**
     * Distance for auto-RPM (from future position if SWM, current if not)
     */
    public double getDistanceForRPM() {
        Pose pose = enabled ? getFuturePose() : follower.getPose();
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * AUTO-SHOOT TRIGGER
     * Returns true when all conditions met:
     * - SWM enabled
     * - Robot in shoot zone
     * - Turret aligned
     * - Shooter at speed
     */
    public boolean shouldAutoShoot() {
        if (!enabled) return false;

        Pose currentPose = follower.getPose();
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        boolean inZone = robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);

        return inZone && turret.isAligned() && shooter.isAtSpeed();
    }

    /**
     * Check if currently in a shoot zone
     */
    public boolean isInShootZone() {
        Pose currentPose = follower.getPose();
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        return robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public boolean isHeadingLockActive() {
        return headingLockActive;
    }

    public double getVelocityMagnitude() {
        Vector vel = follower.getVelocity();
        return Math.hypot(vel.getXComponent(), vel.getYComponent());
    }
}