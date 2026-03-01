package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoPositionSubsystem {

    // ─── State machine ────────────────────────────────────────────────────────
    private enum State {
        IDLE,
        MOVING,           // single-path move (close/far/park)
        GATE_OPEN,        // leg 1: curved approach to gate mouth
        GATE_DWELL,       // 40 ms pause at gate mouth
        GATE_INTAKE       // leg 2: straight push through gate
    }

    private State state = State.IDLE;

    private static final double GATE_DWELL_MS   = 40.0;
    private static final double PATH_TIMEOUT_MS = 5000.0;

    private final Follower    follower;
    private final boolean     isRed;
    private final ElapsedTime timer = new ElapsedTime();

    // ─── Poses ────────────────────────────────────────────────────────────────
    private static final Pose BLUE_PARK        = new Pose(105.103, 32.938, Math.toRadians(90));
    private static final Pose BLUE_CLOSE_SHOOT = new Pose(57.766,  86.731, Math.toRadians(134));
    private static final Pose BLUE_FAR_SHOOT   = new Pose(57,      12,     Math.toRadians(111));

    private static final Pose RED_PARK        = BLUE_PARK.mirror();
    private static final Pose RED_CLOSE_SHOOT = BLUE_CLOSE_SHOOT.mirror();
    private static final Pose RED_FAR_SHOOT   = BLUE_FAR_SHOOT.mirror();

    // ─── Gate cycle poses ─────────────────────────────────────────────────────
    // Red
    private static final Pose RED_GATE_START  = new Pose(131.000, 59.474, Math.toRadians(25));
    private static final Pose RED_GATE_END    = new Pose(131.000, 52.842, Math.toRadians(60));

    // Blue
    private static final Pose BLUE_GATE_START = new Pose(13.000, 59.474, Math.toRadians(155));
    private static final Pose BLUE_GATE_END   = new Pose(13.000, 52.842, Math.toRadians(120));

    public AutoPositionSubsystem(Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed    = isRed;
    }

    // ─── Public API ───────────────────────────────────────────────────────────

    public void goToCloseShoot() {
        Pose target = isRed ? RED_CLOSE_SHOOT : BLUE_CLOSE_SHOOT;
        follower.followPath(buildPath(target));
        state = State.MOVING;
        timer.reset();
    }

    public void goToFarShoot() {
        Pose target = isRed ? RED_FAR_SHOOT : BLUE_FAR_SHOOT;
        follower.followPath(buildPath(target));
        state = State.MOVING;
        timer.reset();
    }

    public void goToPark() {
        Pose target = isRed ? RED_PARK : BLUE_PARK;
        follower.followPath(buildPath(target));
        state = State.MOVING;
        timer.reset();
    }

    /**
     * Gate cycle – two-leg sequence with a 40 ms dwell between legs.
     * Cancellable at any point via cancel().
     * Intake stays ON before/during/after – TeleOp's gateCycleActive handles that.
     *
     * Red:  curve (87,77) → (102,64) → (131,59.474)  heading 47°→25°
     *       line  (131,59.474) → (131,52.842)         heading 25°→60°
     *
     * Blue: curve (57,77) → (42,64) → (13,59.474)    heading 133°→155°
     *       line  (13,59.474) → (13,52.842)           heading 155°→120°
     */
    public void goToGateCycle() {
        follower.followPath(buildGateOpenPath(), true);
        state = State.GATE_OPEN;
        timer.reset();
    }

    /** Cancel any active movement immediately and return to teleop drive. */
    public void cancel() {
        if (state != State.IDLE) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            state = State.IDLE;
        }
    }

    public boolean isActive() {
        return state != State.IDLE;
    }

    // ─── update() – call every loop ───────────────────────────────────────────

    public void update() {
        switch (state) {

            case IDLE:
                break;

            case MOVING:
                if (!follower.isBusy() || timer.milliseconds() > PATH_TIMEOUT_MS) {
                    state = State.IDLE;
                }
                break;

            case GATE_OPEN:
                if (!follower.isBusy() || timer.milliseconds() > PATH_TIMEOUT_MS) {
                    // Arrived at gate mouth – begin 40 ms dwell
                    state = State.GATE_DWELL;
                    timer.reset();
                }
                break;

            case GATE_DWELL:
                // Robot holds position while gate opens
                if (timer.milliseconds() >= GATE_DWELL_MS) {
                    follower.followPath(buildGateIntakePath(), true);
                    state = State.GATE_INTAKE;
                    timer.reset();
                }
                break;

            case GATE_INTAKE:
                if (!follower.isBusy() || timer.milliseconds() > PATH_TIMEOUT_MS) {
                    state = State.IDLE;
                    // gateCycleActive in TeleOp keeps intake running after this
                }
                break;
        }
    }

    // ─── Path builders ────────────────────────────────────────────────────────

    private PathChain buildPath(Pose target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), target)))
                .setConstantHeadingInterpolation(target.getHeading())
                .build();
    }

    private PathChain buildGateOpenPath() {
        if (isRed) {
            return follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(
                            new Pose(87.053,  77.211),
                            new Pose(101.974, 63.921),
                            new Pose(131.000, 59.474)
                    )))
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(25))
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(
                            new Pose(56.947, 77.211),
                            new Pose(42.026, 63.921),
                            new Pose(13.000, 59.474)
                    )))
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(155))
                    .build();
        }
    }

    private PathChain buildGateIntakePath() {
        if (isRed) {
            return follower.pathBuilder()
                    .addPath(new Path(new BezierLine(RED_GATE_START, RED_GATE_END)))
                    .setLinearHeadingInterpolation(
                            RED_GATE_START.getHeading(),
                            RED_GATE_END.getHeading()
                    )
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(new Path(new BezierLine(BLUE_GATE_START, BLUE_GATE_END)))
                    .setLinearHeadingInterpolation(
                            BLUE_GATE_START.getHeading(),
                            BLUE_GATE_END.getHeading()
                    )
                    .build();
        }
    }
}