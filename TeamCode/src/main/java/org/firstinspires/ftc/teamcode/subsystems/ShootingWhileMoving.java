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
     * Time (seconds) from shoot command to disc leaving the flywheel.
     * 3 balls in 1.5s = 0.5s per ball. Transfer time is the mechanical
     * delay before the disc actually exits.
     *
     * INCREASE if shots land behind where you were (you moved past aim point).
     * DECREASE if shots land in front (overcorrecting).
     */
    private static final double BALL_TRANSFER_TIME = 0.40;

    /**
     * Low-pass filter weight for acceleration + omega smoothing.
     *
     *   smoothed = ALPHA * old + (1 - ALPHA) * newMeasurement
     *
     * 0.0 = raw/noisy, 0.9 = heavy smoothing. Start at 0.80.
     */
    private static final double ACCEL_ALPHA = 0.80;

    /**
     * Turret physical offset from robot center of rotation (inches, robot frame).
     * +X = forward on robot, +Y = left on robot.
     *
     * HOW TO MEASURE:
     *   1. Place robot on field, mark center of rotation on the ground
     *      (spin robot in place — the point that doesn't move is the CoR).
     *   2. Measure straight-line distance forward/back → X offset.
     *   3. Measure straight-line distance left/right   → Y offset.
     *   Set to (0, 0) until you measure it — effect is small at low rotation speed.
     */
    private static final double TURRET_OFFSET_X = 0.0;  // TODO: measure
    private static final double TURRET_OFFSET_Y = 0.0;  // TODO: measure

    // =========================================================================
    //  GOAL POSITIONS (inches)
    // =========================================================================
    private static final double BLUE_GOAL_X = 7.0;
    private static final double BLUE_GOAL_Y = 141.0;
    private static final double RED_GOAL_X  = 134.5;
    private static final double RED_GOAL_Y  = 140.0;

    // =========================================================================
    //  VELOCITY TRACKING  (linear + heading for omega estimation)
    // =========================================================================
    private final ElapsedTime accelTimer = new ElapsedTime();
    private double lastVelX    = 0.0;
    private double lastVelY    = 0.0;
    private double lastHeading = 0.0;

    // Smoothed acceleration
    private double accelX = 0.0;
    private double accelY = 0.0;

    // Smoothed angular velocity (rad/s) — estimated by heading finite-difference
    private double omega = 0.0;

    // =========================================================================
    //  STATE
    // =========================================================================
    private boolean enabled              = false;
    private boolean headingLockActive    = false;
    private double  targetHeading        = 0.0;
    private double  turretAngularVelocity = 0.0;  // feedforward for turret PD
    private double  virtualGoalDistance  = 0.0;   // cached for getDistanceForRPM()

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
     * Call every loop. Updates motion estimates, then runs the iterative
     * virtual goal solver to find where to aim and how far the disc will travel.
     */
    public void update() {
        updateMotionEstimates();

        if (!enabled) {
            headingLockActive = false;
            return;
        }

        solveVirtualGoal();
        headingLockActive = true;
    }

    // =========================================================================
    //  MOTION ESTIMATES — acceleration (linear) + omega (angular)
    // =========================================================================

    private void updateMotionEstimates() {
        double dt = accelTimer.seconds();
        if (dt < 0.005) return;

        Vector vel     = follower.getVelocity();
        double vx      = vel.getXComponent();
        double vy      = vel.getYComponent();
        double heading = follower.getPose().getHeading();

        // Linear acceleration — low-pass filtered finite difference
        double rawAccelX = (vx - lastVelX) / dt;
        double rawAccelY = (vy - lastVelY) / dt;
        accelX = ACCEL_ALPHA * accelX + (1.0 - ACCEL_ALPHA) * rawAccelX;
        accelY = ACCEL_ALPHA * accelY + (1.0 - ACCEL_ALPHA) * rawAccelY;

        // Angular velocity — heading finite difference, normalized, low-pass filtered
        double dHeading = heading - lastHeading;
        // Normalize to (-π, π) to handle wrap-around
        while (dHeading >  Math.PI) dHeading -= 2 * Math.PI;
        while (dHeading < -Math.PI) dHeading += 2 * Math.PI;
        double rawOmega = dHeading / dt;
        omega = ACCEL_ALPHA * omega + (1.0 - ACCEL_ALPHA) * rawOmega;

        lastVelX    = vx;
        lastVelY    = vy;
        lastHeading = heading;
        accelTimer.reset();
    }

    // =========================================================================
    //  VIRTUAL GOAL SOLVER — 5-iteration convergence
    //
    //  Instead of predicting where the robot goes, we shift the goal backwards
    //  by how far the robot drifts during (BALL_TRANSFER_TIME + flight time).
    //  The turret aims at this "virtual goal" right now.
    //
    //  Each iteration refines the distance estimate → better flight time →
    //  better virtual goal position. Converges in 3-5 iterations.
    // =========================================================================

    private void solveVirtualGoal() {
        Pose   robotPose = follower.getPose();
        Vector vel       = follower.getVelocity();
        double vx        = vel.getXComponent();
        double vy        = vel.getYComponent();
        double heading   = robotPose.getHeading();
        double goalX     = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY     = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;

        // --- Turret position in field frame ---
        // Rotate robot-frame offset into field frame using current heading
        double cosH         = Math.cos(heading);
        double sinH         = Math.sin(heading);
        double turretFieldX = robotPose.getX() + (TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH);
        double turretFieldY = robotPose.getY() + (TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH);

        // --- Effective turret velocity (robot linear + tangential from rotation) ---
        // Tangential velocity due to robot spinning: v_tan = omega × r (cross product)
        // offset in field frame: (offsetFieldX, offsetFieldY)
        double offsetFieldX = turretFieldX - robotPose.getX();
        double offsetFieldY = turretFieldY - robotPose.getY();
        double turretVx     = vx + (-omega * offsetFieldY);  // -ω × r_y
        double turretVy     = vy + ( omega * offsetFieldX);  //  ω × r_x

        // --- Iterative virtual goal ---
        double virtualGoalX = goalX;
        double virtualGoalY = goalY;
        double distInches   = Math.hypot(goalX - turretFieldX, goalY - turretFieldY);

        for (int i = 0; i < 5; i++) {
            double distMeters  = distInches * 0.0254;
            double rpmForDist  = shooter.getRPMForShot(distMeters);
            // Convert RPM to surface speed (inches/sec). Flywheel radius ≈ 1.5 in.
            // v_surface = RPM * 2π * r / 60
            double shotVelIps  = rpmForDist * (2.0 * Math.PI * 1.5) / 60.0;

            double flightTime  = (shotVelIps > 0.1) ? (distInches / shotVelIps) : 0.0;
            double totalDrift  = BALL_TRANSFER_TIME + flightTime;

            // Shift goal backwards by how far the turret drifts during total drift time
            virtualGoalX = goalX - turretVx * totalDrift;
            virtualGoalY = goalY - turretVy * totalDrift;
            distInches   = Math.hypot(virtualGoalX - turretFieldX, virtualGoalY - turretFieldY);
        }

        // --- Target heading = angle from turret to virtual goal ---
        double dx = virtualGoalX - turretFieldX;
        double dy = virtualGoalY - turretFieldY;
        targetHeading       = Math.atan2(dy, dx);
        virtualGoalDistance = distInches * 0.0254;  // store in meters for RPM lookup

        // --- Turret angular velocity feedforward ---
        // How fast the line-of-sight to virtual goal is rotating
        double distSq = dx * dx + dy * dy;
        if (distSq > 0.1) {
            // ω_los = (v_y * dx - v_x * dy) / dist²   (cross product / dist²)
            double omegaLOS    = (turretVy * dx - turretVx * dy) / distSq;
            turretAngularVelocity = omegaLOS - omega;  // subtract robot rotation → turret-relative
        } else {
            turretAngularVelocity = 0.0;
        }
    }

    // =========================================================================
    //  FUTURE POSE — kept for telemetry / turret teleop compatibility
    //  Now uses the virtual goal drift time rather than raw FLIGHT_TIME_SECONDS
    // =========================================================================

    public Pose getFuturePose() {
        Pose   current = follower.getPose();
        Vector vel     = follower.getVelocity();
        double t       = BALL_TRANSFER_TIME + 0.35;  // approx total drift for display

        return new Pose(
                current.getX() + (vel.getXComponent() * t) + (0.5 * accelX * t * t),
                current.getY() + (vel.getYComponent() * t) + (0.5 * accelY * t * t),
                current.getHeading()
        );
    }

    // =========================================================================
    //  DISTANCE — virtual goal distance when SWM on, real distance when off
    // =========================================================================

    public double getDistanceForRPM() {
        if (enabled) return virtualGoalDistance;
        Pose   pose  = follower.getPose();
        double goalX = isRed ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = isRed ? RED_GOAL_Y : BLUE_GOAL_Y;
        return Math.hypot(goalX - pose.getX(), goalY - pose.getY()) * 0.0254;
    }

    // =========================================================================
    //  READY CHECK
    // =========================================================================

    public boolean isReadyToShoot() {
        if (!enabled) return false;
        return shooter.isAtSpeed() && ((turret == null) || turret.isAligned());
    }

    // =========================================================================
    //  GETTERS — all same names as before, plus turretAngularVelocity
    // =========================================================================

    public boolean isEnabled()              { return enabled; }
    public boolean isHeadingLockActive()    { return headingLockActive; }
    public double  getTargetHeading()       { return targetHeading; }
    public double  getAccelX()              { return accelX; }
    public double  getAccelY()              { return accelY; }
    public double  getOmega()               { return omega; }
    public double  getTurretAngularVelocity() { return turretAngularVelocity; }

    public double getVelocityMagnitude() {
        Vector vel = follower.getVelocity();
        return Math.hypot(vel.getXComponent(), vel.getYComponent());
    }
}