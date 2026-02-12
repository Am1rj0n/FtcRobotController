package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Point;

public class ShootingWhileMoving {

    private final Follower follower;
    private final Shooter shooter;
    private final Turret turret;
    private final boolean isRed;

    private boolean swmEnabled = false;

    // Time for shot to reach goal (seconds)
    private static final double TIME_IN_AIR = 6.7;

    // Shooting zones (from Marrow example)
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

    // Robot zone (14" x 18")
    private final PolygonZone robotZone = new PolygonZone(14, 18);

    // Heading lock state
    private boolean headingLockActive = false;
    private double targetHeading = 0.0;

    // Goal positions
    private static final double BLUE_GOAL_X = 72.0;
    private static final double BLUE_GOAL_Y = 72.0;
    private static final double RED_GOAL_X = 72.0;
    private static final double RED_GOAL_Y = -72.0;

    public ShootingWhileMoving(Follower follower, Shooter shooter, Turret turret, boolean isRed) {
        this.follower = follower;
        this.shooter = shooter;
        this.turret = turret;
        this.isRed = isRed;
    }

    public void toggle() {
        swmEnabled = !swmEnabled;

        if (!swmEnabled) {
            headingLockActive = false;
        }
    }

    public void update() {
        if (!swmEnabled) {
            headingLockActive = false;
            return;
        }

        Pose currentPose = follower.getPose();

        // Update robot zone position and rotation
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        // Check if robot is in a shoot zone
        boolean inShootZone = robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);

        if (inShootZone) {
            // Calculate and apply heading lock
            targetHeading = calculateHeadingToGoal(currentPose);
            headingLockActive = true;
        } else {
            headingLockActive = false;
        }
    }

    /**
     * Get future pose prediction for moving shots
     */
    public Pose getFuturePose() {
        Pose currentPose = follower.getPose();

        // Get velocity components from follower
        double velocityX = follower.getVelocity().getMagnitude() *
                Math.cos(follower.getVelocity().getTheta());
        double velocityY = follower.getVelocity().getMagnitude() *
                Math.sin(follower.getVelocity().getTheta());

        double futureX = currentPose.getX() + (TIME_IN_AIR * velocityX);
        double futureY = currentPose.getY() + (TIME_IN_AIR * velocityY);
        double futureHeading = currentPose.getHeading();

        return new Pose(futureX, futureY, futureHeading);
    }

    /**
     * Calculate required heading to face the goal
     */
    private double calculateHeadingToGoal(Pose pose) {
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();

        return Math.atan2(dy, dx);
    }

    /**
     * Check if all conditions met for auto-shoot
     */
    public boolean shouldAutoShoot() {
        if (!swmEnabled) return false;

        // Update zones
        Pose currentPose = follower.getPose();
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        boolean inZone = robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);

        return inZone && turret.isAligned() && shooter.isAtSpeed();
    }

    public boolean isEnabled() {
        return swmEnabled;
    }

    public boolean isHeadingLockActive() {
        return headingLockActive;
    }

    public double getTargetHeading() {
        return targetHeading;
    }

    public boolean isInShootZone() {
        Pose currentPose = follower.getPose();
        robotZone.setPosition(currentPose.getX(), currentPose.getY());
        robotZone.setRotation(currentPose.getHeading());

        return robotZone.isInside(closeLaunchZone) ||
                robotZone.isInside(farLaunchZone);
    }
}