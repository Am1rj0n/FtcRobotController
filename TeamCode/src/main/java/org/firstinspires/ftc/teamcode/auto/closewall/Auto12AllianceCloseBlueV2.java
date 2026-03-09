package org.firstinspires.ftc.teamcode.auto.closewall;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import java.util.Locale;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Auto12AllianceCloseBlueV2 — PDxRD V2
 * 12-ball alliance close-side autonomous, RED.
 * Starts at (109, 137, 0°).
 *
 * Run order:
 *   INITIAL_SPINUP
 *   → SHOOT0 (preload)
 *   → INTAKEPOS1 → INTAKE1 → GATEPOS1 → GATEOPEN1 → SHOOT1
 *   → INTAKEPOS2 → INTAKE2 → SHOOT2          (no gate for cycle 2)
 *   → GATEPOS3   → GATEOPEN3
 *   → INTAKE3 → INTAKE3_RETRY → SHOOT3
 *   → LEAVE → DONE
 */
@Autonomous(name = "PDxBL V2 12 ball close blue (wall)", group = "PDxBL")
@Configurable
public class Auto12AllianceCloseBlueV2 extends OpMode {

    private static final boolean IS_RED = false;

    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // =========================================================================
    //  TURRET CONSTANTS
    // =========================================================================
    private static final double TURRET_MAX_ANGLE       = 50.0;
    private static final double TURRET_CENTER_POS      = 0.5;
    private static final double TURRET_ALIGN_TOLERANCE = 2.0;
    private static final double TURRET_P               = 0.012;
    private static final double TURRET_D               = 0.001;

    // =========================================================================
    //  SHOOTER RPMs
    // =========================================================================
    private static final double SHOOT_0_RPM = 3150.0;
    private static final double SHOOT_1_RPM = 3150.0;
    private static final double SHOOT_2_RPM = 3150.0;
    private static final double SHOOT_3_RPM = 3150.0;

    // =========================================================================
    //  TIMING CONSTANTS
    // =========================================================================
    private static final double INITIAL_SPINUP_S    = 1.0;
    private static final double SPINUP_MS           = 50.0;
    private static final double SHOOT_MS            = 1500.0;
    /** Max time to wait for alignment before shooting anyway. */
    private static final double ALIGN_TIMEOUT_S     = 1.0;
    private static final double INTAKE_END_DWELL_MS = 120.0;
    /** Pause after gate-open path completes before continuing. */
    private static final double GATE_OPEN_DWELL_MS  = 120.0;

    // =========================================================================
    //  PATH SPEEDS
    // =========================================================================
    private static final double SHOOT_PATH_SPEED  = 0.9;
    private static final double INTAKE_PATH_SPEED = 0.85;
    private static final double GATE_OPEN_SPEED   = 0.8;

    // =========================================================================
    //  PER-PATH TIMEOUTS (seconds)
    // =========================================================================
    private static final double T_SHOOT0       = 4.0;
    private static final double T_INTAKEPOS1   = 2.0;
    private static final double T_INTAKE1      = 3.0;
    private static final double T_GATEPOS1     = 2.0;
    private static final double T_GATEOPEN1    = 2.0;
    private static final double T_SHOOT1       = 3.0;
    private static final double T_INTAKEPOS2   = 2.0;
    private static final double T_INTAKE2      = 3.0;
    private static final double T_SHOOT2       = 3.0;
    private static final double T_GATEPOS3     = 2.0;
    private static final double T_GATEOPEN3    = 2.0;
    private static final double T_INTAKE3      = 3.0;
    private static final double T_INTAKE3RETRY = 3.0;
    private static final double T_SHOOT3       = 3.0;
    private static final double T_LEAVE        = 2.0;

    // =========================================================================
    //  HARDWARE
    // =========================================================================
    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Paths            paths;
    private Intake           intake;
    private Shooter          shooter;
    private Turret           turret;
    private Limelight        limelight;
    private Lights           lights;

    // =========================================================================
    //  TIMERS
    // =========================================================================
    private final ElapsedTime pathTimer      = new ElapsedTime();
    private final ElapsedTime alignTimer     = new ElapsedTime();
    private final ElapsedTime spinupTimer    = new ElapsedTime();
    private final ElapsedTime shootTimer     = new ElapsedTime();
    private final ElapsedTime intakeEndTimer = new ElapsedTime();
    private final ElapsedTime gateOpenTimer  = new ElapsedTime();

    // =========================================================================
    //  TURRET PD STATE
    // =========================================================================
    private double            turretLastError = 0.0;
    private final ElapsedTime turretPidTimer  = new ElapsedTime();
    private double            turretServoPos  = TURRET_CENTER_POS;

    private double currentPathTimeout = 4.0;

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================
    private enum AutoState {
        INITIAL_SPINUP,
        SHOOT0_PATH, SHOOT0_SPINUP, SHOOT0_FIRING,

        // ── Cycle 1: intakepos → intake → gatepos → gateopen → shoot ─────────
        INTAKEPOS1_PATH,
        INTAKE1_PATH, INTAKE1_DWELL,
        GATEPOS1_PATH,
        GATEOPEN1_PATH, GATEOPEN1_DWELL,
        SHOOT1_PATH, SHOOT1_SPINUP, SHOOT1_FIRING,

        // ── Cycle 2: intakepos → intake → shoot (no gate) ────────────────────
        INTAKEPOS2_PATH,
        INTAKE2_PATH, INTAKE2_DWELL,
        SHOOT2_PATH, SHOOT2_SPINUP, SHOOT2_FIRING,

        // ── Cycle 3: gatepos → gateopen → intake → retry → shoot ─────────────
        GATEPOS3_PATH,
        GATEOPEN3_PATH, GATEOPEN3_DWELL,
        INTAKE3_PATH, INTAKE3_DWELL,
        INTAKE3RETRY_PATH, INTAKE3RETRY_DWELL,
        SHOOT3_PATH, SHOOT3_SPINUP, SHOOT3_FIRING,

        LEAVE,
        DONE
    }
    private AutoState state = AutoState.INITIAL_SPINUP;

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(35.000, 137.000, Math.toRadians(180)));
        limelight = new Limelight(hardwareMap, IS_RED);
        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, IS_RED);
        lights    = new Lights(hardwareMap);
        paths     = new Paths(follower);
        panelsTelemetry.debug("Status", "PDxBL V2 - Ready");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
        shooter.setCloseMode();
        intake.setMode(Intake.Mode.INTAKE);
        turretServoPos  = TURRET_CENTER_POS;
        turretLastError = 0.0;
        turretPidTimer.reset();
        spinupTimer.reset();
        state = AutoState.INITIAL_SPINUP;
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        turret.update(follower.getPose());
        updateTurretOdom();
        lights.update(shooter.isActive(), shooter.getRPMMode(), isAligned(), 0);
        autonomousUpdate();
        AutoToTeleTransfer.finalPose = follower.getPose();

        panelsTelemetry.debug("State",        state.name());
        panelsTelemetry.debug("T",            String.format(Locale.US, "%.3f", follower.getCurrentTValue()));
        panelsTelemetry.debug("PathTimer",    String.format(Locale.US, "%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Shoot",        String.format(Locale.US, "%.0f / %.0f ms", shootTimer.milliseconds(), SHOOT_MS));
        panelsTelemetry.debug("GateTimer ms", String.format(Locale.US, "%.0f / %.0f", gateOpenTimer.milliseconds(), GATE_OPEN_DWELL_MS));
        panelsTelemetry.debug("RPM Tgt",      shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",     shooter.getReadRPM());
        panelsTelemetry.debug("AtSpeed",      shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",       intake.getCurrentMode().toString());
        panelsTelemetry.debug("TurretSrv",    String.format(Locale.US, "%.3f", turretServoPos));
        panelsTelemetry.debug("Aligned",      isTurretAligned());
        panelsTelemetry.debug("X",            follower.getPose().getX());
        panelsTelemetry.debug("Y",            follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        lights.off();
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    // =========================================================================
    //  TURRET ODOM TRACKING
    // =========================================================================
    private void updateTurretOdom() {
        Pose p = follower.getPose();
        double angle = Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()))
                - Math.toDegrees(p.getHeading());
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        double current    = servoToAngle(turretServoPos);
        double error      = angle - current;
        double dt         = turretPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - turretLastError) / dt : 0.0;
        turretServoPos    = Math.max(0.0, Math.min(1.0,
                turretServoPos + error * TURRET_P + derivative * TURRET_D));
        turretLastError   = error;
        turretPidTimer.reset();
        turret.setManualAngle(servoToAngle(turretServoPos));
        turret.setMode(Turret.Mode.ODOMETRY);
    }

    private double servoToAngle(double pos) {
        return (pos - TURRET_CENTER_POS) / 0.5 * TURRET_MAX_ANGLE;
    }

    private boolean isTurretAligned() {
        Pose p = follower.getPose();
        double rel = Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()))
                - Math.toDegrees(p.getHeading());
        while (rel >  180) rel -= 360;
        while (rel < -180) rel += 360;
        double desired = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, rel));
        return Math.abs(desired - servoToAngle(turretServoPos)) < TURRET_ALIGN_TOLERANCE;
    }

    private boolean isAligned() {
        return isTurretAligned() || shooter.isAtSpeed();
    }

    // =========================================================================
    //  PATH HELPERS
    // =========================================================================
    private void followShoot(PathChain p, double t)    { follower.setMaxPower(SHOOT_PATH_SPEED);  follower.followPath(p, true);  pathTimer.reset(); currentPathTimeout = t; }
    private void followIntake(PathChain p, double t)   { follower.setMaxPower(INTAKE_PATH_SPEED); follower.followPath(p, false); pathTimer.reset(); currentPathTimeout = t; }
    private void followGateOpen(PathChain p, double t) { follower.setMaxPower(GATE_OPEN_SPEED);   follower.followPath(p, false); pathTimer.reset(); currentPathTimeout = t; }

    private boolean pathDone()      { return follower.getCurrentTValue() >= 0.95 || !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }
    private boolean shootPathDone() { return !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================
    private void autonomousUpdate() {
        switch (state) {

            // ── INITIAL SPINUP ────────────────────────────────────────────────
            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_S) {
                    followShoot(paths.shoot0, T_SHOOT0);
                    state = AutoState.SHOOT0_PATH;
                }
                break;

            case SHOOT0_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT0_SPINUP; }
                break;

            case SHOOT0_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT);
                    shootTimer.reset();
                    state = AutoState.SHOOT0_FIRING;
                }
                break;

            case SHOOT0_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_1_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.intakepos1, T_INTAKEPOS1);
                    state = AutoState.INTAKEPOS1_PATH;
                }
                break;

            // ── CYCLE 1: intakepos → intake → gatepos → gateopen → shoot ─────
            case INTAKEPOS1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    followIntake(paths.intake1, T_INTAKE1);
                    state = AutoState.INTAKE1_PATH;
                }
                break;

            case INTAKE1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE1_DWELL; }
                break;

            case INTAKE1_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followGateOpen(paths.gatepos1, T_GATEPOS1);
                    state = AutoState.GATEPOS1_PATH;
                }
                break;

            case GATEPOS1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    followGateOpen(paths.gateopen1, T_GATEOPEN1);
                    state = AutoState.GATEOPEN1_PATH;
                }
                break;

            case GATEOPEN1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateOpenTimer.reset();
                    state = AutoState.GATEOPEN1_DWELL;
                }
                break;

            case GATEOPEN1_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    followShoot(paths.shoot1, T_SHOOT1);
                    state = AutoState.SHOOT1_PATH;
                }
                break;

            case SHOOT1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT1_SPINUP; }
                break;

            case SHOOT1_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT);
                    shootTimer.reset();
                    state = AutoState.SHOOT1_FIRING;
                }
                break;

            case SHOOT1_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_2_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.intakepos2, T_INTAKEPOS2);
                    state = AutoState.INTAKEPOS2_PATH;
                }
                break;

            // ── CYCLE 2: intakepos → intake → shoot (no gate) ────────────────
            case INTAKEPOS2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    followIntake(paths.intake2, T_INTAKE2);
                    state = AutoState.INTAKE2_PATH;
                }
                break;

            case INTAKE2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE2_DWELL; }
                break;

            case INTAKE2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followShoot(paths.shoot2, T_SHOOT2);
                    state = AutoState.SHOOT2_PATH;
                }
                break;

            case SHOOT2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT2_SPINUP; }
                break;

            case SHOOT2_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT);
                    shootTimer.reset();
                    state = AutoState.SHOOT2_FIRING;
                }
                break;

            case SHOOT2_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_3_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    followGateOpen(paths.gatepos3, T_GATEPOS3);
                    state = AutoState.GATEPOS3_PATH;
                }
                break;

            // ── CYCLE 3: gatepos → gateopen → intake → retry → shoot ─────────
            case GATEPOS3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    followGateOpen(paths.gateopen3, T_GATEOPEN3);
                    state = AutoState.GATEOPEN3_PATH;
                }
                break;

            case GATEOPEN3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateOpenTimer.reset();
                    state = AutoState.GATEOPEN3_DWELL;
                }
                break;

            case GATEOPEN3_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    followIntake(paths.intake3, T_INTAKE3);
                    state = AutoState.INTAKE3_PATH;
                }
                break;

            case INTAKE3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE3_DWELL; }
                break;

            case INTAKE3_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followIntake(paths.intake3retry, T_INTAKE3RETRY);
                    state = AutoState.INTAKE3RETRY_PATH;
                }
                break;

            case INTAKE3RETRY_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { intakeEndTimer.reset(); state = AutoState.INTAKE3RETRY_DWELL; }
                break;

            case INTAKE3RETRY_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    followShoot(paths.shoot3, T_SHOOT3);
                    state = AutoState.SHOOT3_PATH;
                }
                break;

            case SHOOT3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); state = AutoState.SHOOT3_SPINUP; }
                break;

            case SHOOT3_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                alignTimer.reset();
                if (spinupTimer.milliseconds() >= SPINUP_MS
                        && (isAligned() || alignTimer.seconds() >= ALIGN_TIMEOUT_S)) {
                    intake.setMode(Intake.Mode.SHOOT);
                    shootTimer.reset();
                    state = AutoState.SHOOT3_FIRING;
                }
                break;

            case SHOOT3_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    intake.setMode(Intake.Mode.INTAKE);
                    followIntake(paths.leave, T_LEAVE);
                    state = AutoState.LEAVE;
                }
                break;

            // ── LEAVE + DONE ──────────────────────────────────────────────────
            case LEAVE:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { state = AutoState.DONE; }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // =========================================================================
    //  PATHS
    // =========================================================================
    public static class Paths {
        public PathChain shoot0;
        public PathChain intakepos1, intake1, gatepos1, gateopen1, shoot1;
        public PathChain intakepos2, intake2, shoot2;
        public PathChain gatepos3, gateopen3, intake3, intake3retry, shoot3;
        public PathChain leave;

        public Paths(Follower f) {

            // ── SHOOT 0: start → shoot position ──────────────────────────────
            shoot0 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(35.000, 137.000), new Pose(55.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // ── CYCLE 1 ───────────────────────────────────────────────────────
            intakepos1 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 84.000), new Pose(45.526, 84.158)))
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            intake1 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(45.526, 84.158), new Pose(19.579, 84.421)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gatepos1 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(19.579, 84.421), new Pose(25.447, 75.737)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            gateopen1 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(25.447, 74.737), new Pose(15.671, 75.711)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shoot1 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(15.671, 74.711), new Pose(55.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // ── CYCLE 2 (no gate) ─────────────────────────────────────────────
            intakepos2 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 84.000), new Pose(48.158, 59.263)))
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            intake2 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(48.158, 59.263), new Pose(17.368, 59.237)))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot2 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(17.368, 59.237), new Pose(55.000, 84.105)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // ── CYCLE 3: gate first, then intake + retry ──────────────────────
            gatepos3 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 84.105), new Pose(27.474, 70.368)))
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            gateopen3 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(27.474, 66.368), new Pose(14.474, 70.211)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            intake3 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(14.474, 70.211),
                            new Pose(26.632, 55.842),
                            new Pose(8.263, 54.421)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake3retry = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(8.263, 54.421),
                            new Pose(30.684, 44.000),
                            new Pose(8.421, 43.474)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot3 = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(8.421, 43.474), new Pose(54.789, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            // ── LEAVE ─────────────────────────────────────────────────────────
            leave = f.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.789, 84.000), new Pose(43.947, 76.789)))
                    .setConstantHeadingInterpolation(Math.toRadians(130))
                    .build();
        }
    }
}