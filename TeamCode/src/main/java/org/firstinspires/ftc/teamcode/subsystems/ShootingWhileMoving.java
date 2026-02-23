package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingWhileMoving {

    //hi william
    private final Follower follower;
    private final Shooter  shooter;
    private final Turret   turret;
    private final boolean  isRed;

    // ==================== TUNING ====================
    private static final double FLIGHT_TIME_SECONDS = 0.5;

    // ==================== GOAL POSITIONS ====================
    private static final double BLUE_GOAL_X = 0.0;
    private static final double BLUE_GOAL_Y = 144.0;
    private static final double RED_GOAL_X  = 144.0;
    private static final double RED_GOAL_Y  = 144.0;

    // ==================== ACCELERATION TRACKING ====================
    private final ElapsedTime accelTimer = new ElapsedTime();
    private double lastVelX = 0.0;
    private double lastVelY = 0.0;
    private double accelX   = 0.0;
    private double accelY   = 0.0;

    private boolean enabled           = false;
    private boolean headingLockActive = false;
    private double  targetHeading     = 0.0;

    public ShootingWhileMoving(Follower follower, Shooter shooter, Turret turret, boolean isRed) {
        this.follower = follower;
        this.shooter  = shooter;
        this.turret   = turret;
        this.isRed    = isRed;
    }

    public void toggle() {
        enabled = !enabled;
        if (!enabled) headingLockActive = false;
    }

    public void update() {
        updateAcceleration();

        if (!enabled) {
            headingLockActive = false;
            return;
        }

        Pose   futurePose = getFuturePose();
        double goalX      = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY      = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        targetHeading     = Math.atan2(goalY - futurePose.getY(), goalX - futurePose.getX());
        headingLockActive = true;
    }

    private void updateAcceleration() {
        double dt = accelTimer.seconds();
        if (dt < 0.005) return;

        Vector vel = follower.getVelocity();
        double vx  = vel.getXComponent();
        double vy  = vel.getYComponent();

        accelX = (vx - lastVelX) / dt;
        accelY = (vy - lastVelY) / dt;

        lastVelX = vx;
        lastVelY = vy;
        accelTimer.reset();
    }

    //she gon call me baby boo x(t) = x0 + v*t + 0.5*a*t^2

    public Pose getFuturePose() {
        Pose   current = follower.getPose();
        Vector vel     = follower.getVelocity();
        double t       = FLIGHT_TIME_SECONDS;

        return new Pose(
                current.getX() + (vel.getXComponent() * t) + (0.5 * accelX * t * t),
                current.getY() + (vel.getYComponent() * t) + (0.5 * accelY * t * t),
                current.getHeading()
        );
    }

    public double getDistanceForRPM() {
        Pose   pose  = enabled ? getFuturePose() : follower.getPose();
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;
        return Math.hypot(goalX - pose.getX(), goalY - pose.getY());
    }

    public boolean isReadyToShoot() {
        if (!enabled) return false;
        boolean shooterReady = shooter.isAtSpeed();
        boolean turretReady  = (turret == null) || turret.isAligned();
        return shooterReady && turretReady;
    }

    public boolean isEnabled()           { return enabled; }
    public boolean isHeadingLockActive() { return headingLockActive; }
    public double  getTargetHeading()    { return targetHeading; }
    public double  getAccelX()           { return accelX; }
    public double  getAccelY()           { return accelY; }

    public double getVelocityMagnitude() {
        Vector vel = follower.getVelocity();
        return Math.hypot(vel.getXComponent(), vel.getYComponent());
    }
}