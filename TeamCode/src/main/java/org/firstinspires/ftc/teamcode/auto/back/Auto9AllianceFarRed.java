package org.firstinspires.ftc.teamcode.auto.back;

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
 * Auto9AllianceFarRed — 9-ball alliance far-side autonomous, RED.
 * Mirror of Auto9AllianceFarBlue: x_red = 144 - x_blue, start (88, 8, 90deg).
 */
@Autonomous(name = "Auto 9 Alliance Far Red", group = "Alliance")
@Configurable
public class Auto9AllianceFarRed extends OpMode {

    private static final boolean IS_RED = true;

    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y  = 138.0;

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
    private static final double SHOOT_0_RPM = 3900.0;
    private static final double SHOOT_1_RPM = 3900.0;
    private static final double SHOOT_2_RPM = 3900.0;
    private static final double SHOOT_3_RPM = 3900.0;

    // =========================================================================
    //  TIMING CONSTANTS
    // =========================================================================
    private static final double INITIAL_SPINUP_S    = 2.0;
    private static final double SPINUP_MS           = 250.0;
    private static final double SHOOT_MS            = 1800.0;
    /** Max time to wait for alignment before shooting anyway. */
    private static final double ALIGN_TIMEOUT_S     = 1.2;
    private static final double INTAKE_END_DWELL_MS = 100.0;

    // =========================================================================
    //  PATH SPEEDS
    // =========================================================================
    private static final double SHOOT_PATH_SPEED  = 0.8;
    private static final double INTAKE_PATH_SPEED = 0.8;
    private static final double LEAVE_PATH_SPEED  = 1.0;

    // =========================================================================
    //  PER-PATH TIMEOUTS (seconds)
    // =========================================================================
    private static final double T_SHOOT0  = 4.0;
    private static final double T_INTAKE1 = 4.0;
    private static final double T_SHOOT1  = 4.0;
    private static final double T_INTAKE2 = 4.0;
    private static final double T_SHOOT2  = 4.0;
    private static final double T_INTAKE3 = 4.0;
    private static final double T_SHOOT3  = 4.0;
    private static final double T_LEAVE   = 4.0;

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
        INTAKE1_PATH, INTAKE1_DWELL,
        SHOOT1_PATH, SHOOT1_SPINUP, SHOOT1_FIRING,
        INTAKE2_PATH, INTAKE2_DWELL,
        SHOOT2_PATH, SHOOT2_SPINUP, SHOOT2_FIRING,
        INTAKE3_PATH, INTAKE3_DWELL,
        SHOOT3_PATH, SHOOT3_SPINUP, SHOOT3_FIRING,
        LEAVE_PATH, DONE
    }
    private AutoState state = AutoState.INITIAL_SPINUP;

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        // Blue starts at (56, 8, 90°) → Red mirrors to (88, 8, 90°)
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        limelight = new Limelight(hardwareMap, IS_RED);
        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        turret    = new Turret(hardwareMap, limelight, IS_RED);
        lights    = new Lights(hardwareMap);
        paths     = new Paths(follower);
        panelsTelemetry.debug("Status", "Auto 9 Alliance Far Red - Ready");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
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
        panelsTelemetry.debug("State",     state.name());
        panelsTelemetry.debug("T",         String.format(Locale.US, "%.3f", follower.getCurrentTValue()));
        panelsTelemetry.debug("PathTimer", String.format(Locale.US, "%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Shoot",     String.format(Locale.US, "%.0f / %.0f ms", shootTimer.milliseconds(), SHOOT_MS));
        panelsTelemetry.debug("RPM Tgt",   shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",  shooter.getReadRPM());
        panelsTelemetry.debug("AtSpeed",   shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",    intake.getCurrentMode().toString());
        panelsTelemetry.debug("TurretSrv", String.format(Locale.US, "%.3f", turretServoPos));
        panelsTelemetry.debug("Aligned",   isTurretAligned());
        panelsTelemetry.debug("X",         follower.getPose().getX());
        panelsTelemetry.debug("Y",         follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.stop(); intake.stop(); lights.off();
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

    // =========================================================================
    //  PATH HELPERS
    // =========================================================================
    private void followShoot(PathChain p, double t)  { follower.setMaxPower(SHOOT_PATH_SPEED);  follower.followPath(p, true);  pathTimer.reset(); currentPathTimeout = t; }
    private void followIntake(PathChain p, double t) { follower.setMaxPower(INTAKE_PATH_SPEED); follower.followPath(p, false); pathTimer.reset(); currentPathTimeout = t; }
    private void followLeave(PathChain p, double t)  { follower.setMaxPower(LEAVE_PATH_SPEED);  follower.followPath(p, false); pathTimer.reset(); currentPathTimeout = t; }

    private boolean pathDone()      { return follower.getCurrentTValue() >= 0.95 || !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }
    private boolean shootPathDone() { return !follower.isBusy() || pathTimer.seconds() >= currentPathTimeout; }

    private boolean isAligned() {
        return isTurretAligned() || shooter.isAtSpeed();
    }

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================
    private void autonomousUpdate() {
        switch (state) {

            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_S) {
                    followShoot(paths.shoot0, T_SHOOT0); state = AutoState.SHOOT0_PATH;
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
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT0_FIRING;
                }
                break;

            case SHOOT0_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_1_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
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
                    followShoot(paths.shoot1, T_SHOOT1); state = AutoState.SHOOT1_PATH;
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
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT1_FIRING;
                }
                break;

            case SHOOT1_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_2_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
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
                    followShoot(paths.shoot2, T_SHOOT2); state = AutoState.SHOOT2_PATH;
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
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT2_FIRING;
                }
                break;

            case SHOOT2_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    shooter.setTargetRPM(SHOOT_3_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
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
                    followShoot(paths.shoot3, T_SHOOT3); state = AutoState.SHOOT3_PATH;
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
                    intake.setMode(Intake.Mode.SHOOT); shootTimer.reset(); state = AutoState.SHOOT3_FIRING;
                }
                break;

            case SHOOT3_FIRING:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.milliseconds() >= SHOOT_MS) {
                    intake.setMode(Intake.Mode.INTAKE);
                    followLeave(paths.Leave, T_LEAVE);
                    state = AutoState.LEAVE_PATH;
                }
                break;

            case LEAVE_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { state = AutoState.DONE; }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // =========================================================================
    //  PATHS — Red (x_red = 144 - x_blue, headings tangent-flipped)
    //
    //  Blue start: (56, 8, 90°) → Red start: (88, 8, 90°)
    // =========================================================================
    public static class Paths {
        public PathChain shoot0, intake1, shoot1, intake2, shoot2, intake3, shoot3, Leave;

        public Paths(Follower f) {
            // Blue: (56,8) → (56,16.895)
            // Red:  (88,8) → (88,16.895)
            shoot0 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(88.000, 16.895)))
                    .setTangentHeadingInterpolation().build();

            // Blue intake1: (56,16.895) curve → (39.605,33.579) → (13.316,35.421)
            // Red:          (88,16.895) curve → (104.395,33.579) → (130.684,35.421)
            intake1 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 88.000, 16.895), new Pose( 86.605, 41.842),
                            new Pose(104.395, 33.579), new Pose(130.684, 35.421)))
                    .setTangentHeadingInterpolation().build();

            // Blue shoot1: (13.316,35.421) curve → (46.474,33.079) → (47.711,25.553) → (57.842,12.211) reversed
            // Red:         (130.684,35.421) curve → (97.526,33.079) → (96.289,25.553) → (86.158,12.211) reversed
            shoot1 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.684, 35.421), new Pose( 97.526, 33.079),
                            new Pose( 96.289, 25.553), new Pose( 86.158, 12.211)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // Blue intake2: (57.842,12.211) curve → (38.105,7.947) → (9.158,8.053)
            // Red:          (86.158,12.211) curve → (105.895,7.947) → (134.842,8.053)
            intake2 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose( 86.158, 12.211), new Pose(105.895,  7.947), new Pose(134.842,  8.053)))
                    .setTangentHeadingInterpolation().build();

            // Blue shoot2: (9.158,8.053) curve → (31.184,7.421) → (54.184,17.842) → (57.842,12.053) reversed
            // Red:         (134.842,8.053) curve → (112.816,7.421) → (89.816,17.842) → (86.158,12.053) reversed
            shoot2 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(134.842,  8.053), new Pose(112.816,  7.421),
                            new Pose( 89.816, 17.842), new Pose( 86.158, 12.053)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // Blue intake3: (57.842,12.053) → (9.632,12.158)
            // Red:          (86.158,12.053) → (134.368,12.158)
            intake3 = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(86.158, 12.053), new Pose(134.368, 12.158)))
                    .setTangentHeadingInterpolation().build();

            // Blue shoot3: (9.632,12.158) curve → (56.263,16.368) → (57.526,13.579) reversed
            // Red:         (134.368,12.158) curve → (87.737,16.368) → (86.474,13.579) reversed
            shoot3 = f.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(134.368, 12.158), new Pose(87.737, 16.368), new Pose(86.474, 13.579)))
                    .setTangentHeadingInterpolation().setReversed().build();

            // Blue leave: (57.526,13.579) → (38.211,13.421)
            // Red:        (86.474,13.579) → (105.789,13.421)
            Leave = f.pathBuilder()
                    .addPath(new BezierLine(new Pose(86.474, 13.579), new Pose(105.789, 13.421)))
                    .setTangentHeadingInterpolation().build();
        }
    }
}