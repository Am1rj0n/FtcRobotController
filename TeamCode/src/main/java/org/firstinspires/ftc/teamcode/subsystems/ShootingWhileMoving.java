package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingWhileMoving {

    private final Follower follower;
    private final Shooter  shooter;
    private final Turret   turret;
    private final boolean  isRed;

    // =========================================================================
    //  TUNING
    // =========================================================================

    /**
     * How far into the future to predict robot position (seconds).
     *
     * This is the time from disc leaving the flywheel to crossing the goal.
     * Measure with slow-motion video: start timing when disc exits the shooter,
     * stop when it enters the goal opening.
     *
     * INCREASE if shots land short (you moved past where you aimed).
     * DECREASE if shots land long (you're leading too much, correction overshoots).
     *
     * Tune in 0.05s steps. Typical FTC range: 0.3 – 0.8s.
     */
    private static final double FLIGHT_TIME_SECONDS = 0.50;

    /**
     * Low-pass filter weight for acceleration smoothing.
     *
     * Acceleration is estimated by finite differencing (velocity delta / dt),
     * which is noisy — especially at high speed where odometry updates quickly.
     * The filter blends old accel estimate with new measurement each loop:
     *
     *   smoothedAccel = ACCEL_ALPHA * oldAccel + (1 - ACCEL_ALPHA) * newMeasurement
     *
     * ACCEL_ALPHA = 0.0  → no filtering (raw, very noisy at high speed)
     * ACCEL_ALPHA = 0.8  → strong smoothing (recommended starting point)
     * ACCEL_ALPHA = 0.95 → very heavy filtering (slow to respond to real accel changes)
     *
     * If SWM overcorrects at high speed → increase ACCEL_ALPHA toward 0.9.
     * If SWM is too slow to respond to direction changes → decrease toward 0.6.
     */
    private static final double ACCEL_ALPHA = 0.80;

    // =========================================================================
    //  GOAL POSITIONS
    // =========================================================================
    private static final double BLUE_GOAL_X = 0.0;
    private static final double BLUE_GOAL_Y = 144.0;
    private static final double RED_GOAL_X  = 144.0;
    private static final double RED_GOAL_Y  = 144.0;

    // =========================================================================
    //  ACCELERATION TRACKING
    // =========================================================================
    private final ElapsedTime accelTimer = new ElapsedTime();
    private double lastVelX = 0.0;
    private double lastVelY = 0.0;

    // Smoothed (low-pass filtered) acceleration values
    private double accelX = 0.0;
    private double accelY = 0.0;

    // =========================================================================
    //  STATE
    // =========================================================================
    private boolean enabled           = false;
    private boolean headingLockActive = false;
    private double  targetHeading     = 0.0;

    // =========================================================================
    //  CONSTRUCTOR
    // =========================================================================
    public ShootingWhileMoving(Follower follower, Shooter shooter, Turret turret, boolean isRed) {
        this.follower = follower;
        this.shooter  = shooter;
        this.turret   = turret;
        this.isRed    = isRed;
    }

    // =========================================================================
    //  PUBLIC API
    // =========================================================================

    public void toggle() {
        enabled = !enabled;
        if (!enabled) headingLockActive = false;
    }

    /**
     * Call every loop. Updates acceleration estimate with low-pass filter,
     * then computes target heading from predicted future pose.
     */
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

    // =========================================================================
    //  ACCELERATION — with low-pass filter
    // =========================================================================

    private void updateAcceleration() {
        double dt = accelTimer.seconds();
        if (dt < 0.005) return;  // guard against divide-by-zero on first loop

        Vector vel = follower.getVelocity();
        double vx  = vel.getXComponent();
        double vy  = vel.getYComponent();

        // Raw acceleration from finite differencing
        double rawAccelX = (vx - lastVelX) / dt;
        double rawAccelY = (vy - lastVelY) / dt;

        // Low-pass filter: blend old estimate with new measurement.
        // This kills the high-frequency noise that spikes at high speed.
        accelX = ACCEL_ALPHA * accelX + (1.0 - ACCEL_ALPHA) * rawAccelX;
        accelY = ACCEL_ALPHA * accelY + (1.0 - ACCEL_ALPHA) * rawAccelY;

        lastVelX = vx;
        lastVelY = vy;
        accelTimer.reset();
    }

    // =========================================================================
    //  PREDICTION — x(t) = x0 + v*t + 0.5*a*t²
    // =========================================================================

    /**
     * Returns where the robot will be FLIGHT_TIME_SECONDS from now.
     * Uses smoothed acceleration so high-speed noise doesn't blow up the prediction.
     */
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

    // =========================================================================
    //  DISTANCE — uses future pose when SWM on, current pose when off
    // =========================================================================

    public double getDistanceForRPM() {
        Pose   pose  = enabled ? getFuturePose() : follower.getPose();
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;
        return Math.hypot(goalX - pose.getX(), goalY - pose.getY());
    }

    // =========================================================================
    //  READY CHECK
    // =========================================================================

    public boolean isReadyToShoot() {
        if (!enabled) return false;
        boolean shooterReady = shooter.isAtSpeed();
        boolean turretReady  = (turret == null) || turret.isAligned();
        return shooterReady && turretReady;
    }

    // =========================================================================
    //  GETTERS
    // =========================================================================

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