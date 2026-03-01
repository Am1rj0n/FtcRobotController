package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "PINKxROLL Blue", group = "Autonomous")
@Configurable
public class PINKxROLLBlue extends OpMode {

    // =====================================================================
    //  TELEMETRY / HARDWARE
    // =====================================================================
    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Paths            paths;

    private Intake  intake;
    private Shooter shooter;
    private Servo   turretServo;

    // =====================================================================
    //  TIMERS
    // =====================================================================
    private final ElapsedTime shootTimer      = new ElapsedTime();
    private final ElapsedTime pathTimer       = new ElapsedTime();
    private final ElapsedTime spinupTimer     = new ElapsedTime();
    private final ElapsedTime gateWaitTimer   = new ElapsedTime(); // dwell at end of GATEOPEN
    private final ElapsedTime intakeEndTimer  = new ElapsedTime(); // dwell at end of INTAKE paths

    // =====================================================================
    //  TURRET
    // =====================================================================
    private static final double FIXED_TURRET_POSITION = 0.5; // TODO: tune for blue

    // =====================================================================
    //  SHOOTER RPMs  — tune each independently
    // =====================================================================
    private static final double SHOOT_0_RPM = 3800.0;
    private static final double SHOOT_1_RPM = 3800.0;
    private static final double SHOOT_2_RPM = 3800.0;
    private static final double SHOOT_3_RPM = 3800.0;

    // =====================================================================
    //  TIMING CONSTANTS  — all easily tunable here
    // =====================================================================
    private static final double INITIAL_SPINUP_DURATION = 1.0;   // seconds before first path
    private static final double SPINUP_DURATION         = 0.3;   // seconds spinup before each shoot
    private static final double SHOOT_DURATION          = 1.5;   // seconds of active disc feeding

    // How long to pause at the END of each GATEOPEN path before moving on
    private static final double GATE_OPEN_DWELL_MS  = 300.0;  // ms

    // How long to pause at end of each INTAKE path (ball settling time)
    private static final double INTAKE_END_DWELL_MS = 50;    // ms — set > 0 to add dwell

    // =====================================================================
    //  PATH SPEEDS  — tune each class independently
    // =====================================================================
    private static final double SHOOT_PATH_SPEED    = 0.9;    // full speed to shoot positions
    private static final double INTAKE_PATH_SPEED   = 0.85;    // slightly reduced for intaking
    private static final double GATE_PATH_SPEED     = 0.9;    // slower for gate approach + open

    // =====================================================================
    //  PER-PATH TIMEOUTS  — tune each independently (seconds)
    // =====================================================================
    private static final double T_SHOOT0     = 4.0;
    private static final double T_INTAKEPOS1 = 2.0;
    private static final double T_INTAKE1    = 3.0;
    private static final double T_GATEPOS    = 2.0;
    private static final double T_GATEOPEN   = 2.0;
    private static final double T_SHOOT1     = 3.0;
    private static final double T_INTAKEPOS2 = 3.0;
    private static final double T_INTAKE2    = 3.0;
    private static final double T_GATEPOS2   = 2.0;
    private static final double T_GATEOPEN2  = 2.0;
    private static final double T_SHOOT2     = 3.0;
    private static final double T_INTAKEPOS3 = 3.0;
    private static final double T_INTAKE3    = 3.0;
    private static final double T_SHOOT3     = 4.0;

    // =====================================================================
    //  INTERNAL STATE
    // =====================================================================
    private double currentPathTimeout = 4.0;

    // =====================================================================
    //  STATE MACHINE
    // =====================================================================
    private enum AutoState {
        INITIAL_SPINUP,

        // Preload shoot
        SHOOT_0_PATH,
        SHOOT_0_SPINUP,
        SHOOT_0,

        // Cycle 1 — row intake → gate → shoot
        INTAKEPOS1,
        INTAKE1,
        INTAKE1_DWELL,
        GATEPOS,
        GATEOPEN,
        GATEOPEN_DWELL,
        SHOOT1_PATH,
        SHOOT1_SPINUP,
        SHOOT1,

        // Cycle 2 — row intake → gate → shoot
        INTAKEPOS2,
        INTAKE2,
        INTAKE2_DWELL,
        GATEPOS2,
        GATEOPEN2,
        GATEOPEN2_DWELL,
        SHOOT2_PATH,
        SHOOT2_SPINUP,
        SHOOT2,

        // Cycle 3 — long row intake → shoot
        INTAKEPOS3,
        INTAKE3,
        INTAKE3_DWELL,
        SHOOT3_PATH,
        SHOOT3_SPINUP,
        SHOOT3,

        DONE
    }

    private AutoState currentState = AutoState.INITIAL_SPINUP;

    // =====================================================================
    //  LIFECYCLE
    // =====================================================================
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(25.526, 129.737, Math.toRadians(144)));

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(FIXED_TURRET_POSITION);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "PINKxROLL Blue - Ready");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
        turretServo.setPosition(FIXED_TURRET_POSITION);

        spinupTimer.reset();
        currentState = AutoState.INITIAL_SPINUP;
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();

        autonomousUpdate();

        AutoToTeleTransfer.finalPose = follower.getPose();

        panelsTelemetry.debug("State",        currentState.name());
        panelsTelemetry.debug("T Value",      follower.getCurrentTValue());
        panelsTelemetry.debug("Busy",         follower.isBusy());
        panelsTelemetry.debug("Path Timer",   String.format("%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Spinup",       String.format("%.2f / %.1f", spinupTimer.seconds(), SPINUP_DURATION));
        panelsTelemetry.debug("Gate Dwell",   String.format("%.0f / %.0f ms", gateWaitTimer.milliseconds(), GATE_OPEN_DWELL_MS));
        panelsTelemetry.debug("Intake Dwell", String.format("%.0f / %.0f ms", intakeEndTimer.milliseconds(), INTAKE_END_DWELL_MS));
        panelsTelemetry.debug("RPM Target",   shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",     shooter.getReadRPM());
        panelsTelemetry.debug("At Speed",     shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",       intake.getCurrentMode().toString());
        panelsTelemetry.debug("X",            follower.getPose().getX());
        panelsTelemetry.debug("Y",            follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    // =====================================================================
    //  HELPERS
    // =====================================================================

    private void startShootPath(PathChain path, double timeout) {
        follower.setMaxPower(SHOOT_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private void startIntakePath(PathChain path, double timeout) {
        follower.setMaxPower(INTAKE_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private void startGatePath(PathChain path, double timeout) {
        follower.setMaxPower(GATE_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    /** Standard 95% T OR done OR timeout — used for intake and transit paths. */
    private boolean pathDone() {
        return follower.getCurrentTValue() >= 0.95
                || !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    /** Shoot positioning: fully done OR timeout (no T-value shortcut). */
    private boolean shootPathDone() {
        return !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    private void setPreShootRPM(double rpm) {
        shooter.setTargetRPM(rpm);
    }

    // =====================================================================
    //  STATE MACHINE
    // =====================================================================
    private void autonomousUpdate() {
        switch (currentState) {

            // -----------------------------------------------------------------
            // INITIAL SPINUP — 1s flywheel ramp before any movement
            // -----------------------------------------------------------------
            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_DURATION) {
                    startShootPath(paths.SHOOT0, T_SHOOT0);
                    currentState = AutoState.SHOOT_0_PATH;
                }
                break;

            // -----------------------------------------------------------------
            // SHOOT 0 — preload
            // -----------------------------------------------------------------
            case SHOOT_0_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) {
                    spinupTimer.reset();
                    currentState = AutoState.SHOOT_0_SPINUP;
                }
                break;

            case SHOOT_0_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_0;
                }
                break;

            case SHOOT_0:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    setPreShootRPM(SHOOT_1_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.INTAKEPOS1, T_INTAKEPOS1);
                    currentState = AutoState.INTAKEPOS1;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 1 — row intake at Y~84 → gate curve → gate push → shoot
            // -----------------------------------------------------------------
            case INTAKEPOS1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.INTAKE1, T_INTAKE1);
                    currentState = AutoState.INTAKE1;
                }
                break;

            case INTAKE1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    intakeEndTimer.reset();
                    currentState = AutoState.INTAKE1_DWELL;
                }
                break;

            case INTAKE1_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    startGatePath(paths.GATEPOS, T_GATEPOS);
                    currentState = AutoState.GATEPOS;
                }
                break;

            case GATEPOS:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startGatePath(paths.GATEOPEN, T_GATEOPEN);
                    currentState = AutoState.GATEOPEN;
                }
                break;

            case GATEOPEN:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateWaitTimer.reset();
                    currentState = AutoState.GATEOPEN_DWELL;
                }
                break;

            case GATEOPEN_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateWaitTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    startShootPath(paths.SHOOT1, T_SHOOT1);
                    currentState = AutoState.SHOOT1_PATH;
                }
                break;

            case SHOOT1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) {
                    spinupTimer.reset();
                    currentState = AutoState.SHOOT1_SPINUP;
                }
                break;

            case SHOOT1_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT1;
                }
                break;

            case SHOOT1:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    setPreShootRPM(SHOOT_2_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.INTAKEPOS2, T_INTAKEPOS2);
                    currentState = AutoState.INTAKEPOS2;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 2 — row intake at Y~60 → gate curve → gate push → shoot
            // -----------------------------------------------------------------
            case INTAKEPOS2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.INTAKE2, T_INTAKE2);
                    currentState = AutoState.INTAKE2;
                }
                break;

            case INTAKE2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    intakeEndTimer.reset();
                    currentState = AutoState.INTAKE2_DWELL;
                }
                break;

            case INTAKE2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    startGatePath(paths.GATEPOS2, T_GATEPOS2);
                    currentState = AutoState.GATEPOS2;
                }
                break;

            case GATEPOS2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startGatePath(paths.GATEOPEN2, T_GATEOPEN2);
                    currentState = AutoState.GATEOPEN2;
                }
                break;

            case GATEOPEN2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateWaitTimer.reset();
                    currentState = AutoState.GATEOPEN2_DWELL;
                }
                break;

            case GATEOPEN2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateWaitTimer.milliseconds() >= GATE_OPEN_DWELL_MS) {
                    startShootPath(paths.SHOOT2, T_SHOOT2);
                    currentState = AutoState.SHOOT2_PATH;
                }
                break;

            case SHOOT2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) {
                    spinupTimer.reset();
                    currentState = AutoState.SHOOT2_SPINUP;
                }
                break;

            case SHOOT2_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT2;
                }
                break;

            case SHOOT2:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    setPreShootRPM(SHOOT_3_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.INTAKEPOS3, T_INTAKEPOS3);
                    currentState = AutoState.INTAKEPOS3;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 3 — long transit to Y~36 row → straight intake → shoot
            // -----------------------------------------------------------------
            case INTAKEPOS3:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.INTAKE3, T_INTAKE3);
                    currentState = AutoState.INTAKE3;
                }
                break;

            case INTAKE3:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    intakeEndTimer.reset();
                    currentState = AutoState.INTAKE3_DWELL;
                }
                break;

            case INTAKE3_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (intakeEndTimer.milliseconds() >= INTAKE_END_DWELL_MS) {
                    startShootPath(paths.SHOOT3, T_SHOOT3);
                    currentState = AutoState.SHOOT3_PATH;
                }
                break;

            case SHOOT3_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) {
                    spinupTimer.reset();
                    currentState = AutoState.SHOOT3_SPINUP;
                }
                break;

            case SHOOT3_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT3;
                }
                break;

            case SHOOT3:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.INTAKE);
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // =====================================================================
    //  PATHS  (Blue-side — doc 10)
    // =====================================================================
    public static class Paths {
        public PathChain SHOOT0;
        public PathChain INTAKEPOS1;
        public PathChain INTAKE1;
        public PathChain GATEPOS;
        public PathChain GATEOPEN;
        public PathChain SHOOT1;
        public PathChain INTAKEPOS2;
        public PathChain INTAKE2;
        public PathChain GATEPOS2;
        public PathChain GATEOPEN2;
        public PathChain SHOOT2;
        public PathChain INTAKEPOS3;
        public PathChain INTAKE3;
        public PathChain SHOOT3;

        public Paths(Follower follower) {

            // (25.526,129.737,144°) → (53.789,87,134°)
            SHOOT0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(25.526, 129.737),
                            new Pose(53.789,  87.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(134))
                    .build();

            // (53.789,87,134°) → (43.789,84.105,180°)
            INTAKEPOS1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(53.789, 87.000),
                            new Pose(43.789, 84.105)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                    .build();

            // (43.789,84.105,180°) → (15.737,84,180°)
            INTAKE1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(43.789, 84.105),
                            new Pose(15.737, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // (15.737,84,180°) → curve → (19.789,75.053,90°)
            GATEPOS = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.737, 84.000),
                            new Pose(27.289, 80.000),
                            new Pose(19.789, 75.053)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            // (19.789,75.053,90°) → (14.474,75.053,90°)
            GATEOPEN = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(19.789, 75.053),
                            new Pose(14.474, 75.053)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            // (14.474,75.053,90°) → (53.684,86.947,134°)
            SHOOT1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(14.474, 75.053),
                            new Pose(53.684, 86.947)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                    .build();

            // (53.684,86.947,134°) → (45.789,60.421,180°)
            INTAKEPOS2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(53.684, 86.947),
                            new Pose(45.789, 60.421)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                    .build();

            // (45.789,60.421,180°) → (15.895,60.579,180°)
            INTAKE2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(45.789, 60.421),
                            new Pose(15.895, 60.579)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // (15.895,60.579,180°) → curve → (22,71.263,90°)
            GATEPOS2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(15.895, 60.579),
                            new Pose(26.105, 54.026),
                            new Pose(22.000, 71.263)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            // (22,71.263,90°) → (15.474,71,90°)
            GATEOPEN2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.000, 71.263),
                            new Pose(15.474, 71.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            // (15.474,71,90°) → (53.474,87.053,134°)
            SHOOT2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(15.474, 71.000),
                            new Pose(53.474, 87.053)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                    .build();

            // (53.474,87.053,134°) → (35.895,36.158,180°)
            INTAKEPOS3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(53.474, 87.053),
                            new Pose(35.895, 36.158)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                    .build();

            // (35.895,36.158,180°) → (9.263,36.368,180°)
            INTAKE3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(35.895, 36.158),
                            new Pose(9.263,  36.368)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // (9.263,36.368,180°) → (58.105,102.105,141°)
            SHOOT3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.263,   36.368),
                            new Pose(58.105, 102.105)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(141))
                    .build();
        }
    }
}