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

@Autonomous(name = "Auto 15 TURTLE WALKERS", group = "Autonomous")
@Configurable
public class Gate15Blue extends OpMode {

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
    private final ElapsedTime shootTimer     = new ElapsedTime();
    private final ElapsedTime pathTimer      = new ElapsedTime();
    private final ElapsedTime spinupTimer    = new ElapsedTime();
    private final ElapsedTime gateWaitTimer  = new ElapsedTime();  // for gate dwell delays
    private final ElapsedTime gateOpenTimer  = new ElapsedTime();  // for 100ms after GATEOPEN 99%

    // =====================================================================
    //  TURRET
    // =====================================================================
    private static final double FIXED_TURRET_POSITION = 0.5; // TODO: tune for blue

    // =====================================================================
    //  SHOOTER RPMs  — tune each independently
    // =====================================================================
    private static final double SHOOT_0_RPM = 2800.0;
    private static final double SHOOT_1_RPM = 2800.0;
    private static final double SHOOT_2_RPM = 2800.0;
    private static final double SHOOT_3_RPM = 2800.0;
    private static final double SHOOT_4_RPM = 3000.0;

    // =====================================================================
    //  TIMING CONSTANTS
    // =====================================================================
    private static final double INITIAL_SPINUP_DURATION   = 0.5;    // seconds before first path
    private static final double SPINUP_DURATION           = 0.4;    // seconds before each shoot
    private static final double SHOOT_DURATION            = 1.3;    // seconds of active feeding
    private static final double GATE_INTAKE_DWELL_MS      = 600.0; // ms to hold at end of GATEINTAKE
    private static final double GATE_OPEN_DELAY_MS        = 35.0;  // ms to hold after GATEOPEN 99%

    // =====================================================================
    //  PATH SPEEDS
    // =====================================================================
    private static final double INTAKE_PATH_SPEED      = 0.9;
    private static final double GATE_INTAKE_PATH_SPEED = 0.9; // slower for gate intake passes

    // =====================================================================
    //  PER-PATH TIMEOUTS  — tune each independently (seconds)
    // =====================================================================
    private static final double T_SHOOT0        = 4.0;
    private static final double T_INTAKEPOS1    = 2.0;
    private static final double T_INTAKE1       = 3.0;
    private static final double T_SHOOT1        = 3.0;
    private static final double T_GATEOPEN      = 3.0;
    private static final double T_GATEINTAKE    = 2.0;
    private static final double T_SHOOT2        = 3.0;
    private static final double T_GATEINTAKE2   = 3.0;
    private static final double T_SHOOT3        = 3.0;
    private static final double T_INTAKE2       = 3.0;
    private static final double T_SHOOT4        = 3.0;
    private static final double T_PATH12        = 2.0;

    // =====================================================================
    //  INTERNAL STATE
    // =====================================================================
    private double  currentPathTimeout  = 4.0;
    private boolean gateOpenDelayDone   = false; // tracks whether 100ms post-gateopen has elapsed

    // =====================================================================
    //  STATE MACHINE
    // =====================================================================
    private enum AutoState {
        INITIAL_SPINUP,

        SHOOT_0_PATH,
        SHOOT_0_SPINUP,
        SHOOT_0,

        INTAKEPOS1,
        INTAKE1,
        SHOOT1_PATH,
        SHOOT1_SPINUP,
        SHOOT1,

        // Gate cycle 1: approach → wait 100ms → intake down → dwell 1000ms → shoot
        GATEOPEN,
        GATEOPEN_DELAY,   // 100ms hold after 99% of GATEOPEN
        GATEINTAKE,       // slow push down
        GATEINTAKE_DWELL, // 1000ms hold at end of GATEINTAKE
        SHOOT2_PATH,
        SHOOT2_SPINUP,
        SHOOT2,

        // Gate cycle 2: re-approach → wait 100ms → intake down → dwell 1000ms → shoot
        GATEINTAKE2,       // re-approach (second GATEINTAKE path)
        GATEINTAKE2_DELAY, // 100ms hold after 99% of GATEINTAKE2 approach
        GATEINTAKE2_PUSH,  // slow down-push on re-approach (reuse GATEINTAKE path geometry)
        GATEINTAKE2_DWELL, // 1000ms hold
        SHOOT3_PATH,
        SHOOT3_SPINUP,
        SHOOT3,

        INTAKE2,
        SHOOT4_PATH,
        SHOOT4_SPINUP,
        SHOOT4,

        PATH12,
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
        follower.setStartingPose(new Pose(26.316, 128.947, Math.toRadians(144)));

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(FIXED_TURRET_POSITION);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Auto 15 Near Blue - Ready");
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

        panelsTelemetry.debug("State",       currentState.name());
        panelsTelemetry.debug("T Value",     follower.getCurrentTValue());
        panelsTelemetry.debug("Busy",        follower.isBusy());
        panelsTelemetry.debug("Path Timer",  String.format("%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Spinup",      String.format("%.2f / %.1f", spinupTimer.seconds(), SPINUP_DURATION));
        panelsTelemetry.debug("GateWait ms", String.format("%.0f", gateWaitTimer.milliseconds()));
        panelsTelemetry.debug("RPM Target",  shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",    shooter.getReadRPM());
        panelsTelemetry.debug("At Speed",    shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",      intake.getCurrentMode().toString());
        panelsTelemetry.debug("X",           follower.getPose().getX());
        panelsTelemetry.debug("Y",           follower.getPose().getY());
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

    private void startIntakePath(PathChain path, double timeout) {
        follower.setMaxPower(INTAKE_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private void startGateIntakePath(PathChain path, double timeout) {
        follower.setMaxPower(GATE_INTAKE_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private void startShootPath(PathChain path, double timeout) {
        follower.setMaxPower(0.97);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    /** Standard intake completion: 95% T OR done OR timeout. */
    private boolean pathDone() {
        return follower.getCurrentTValue() >= 0.95
                || !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    /** Gate OPEN approach: 99% T OR done OR timeout, then hold 100ms. */
    private boolean gateOpenPathDone() {
        return follower.getCurrentTValue() >= 0.99
                || !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    /** Shoot positioning: fully done OR timeout. */
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
            // INITIAL SPINUP
            // -----------------------------------------------------------------
            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_DURATION) {
                    startShootPath(paths.SHOOT0, T_SHOOT0);
                    currentState = AutoState.SHOOT_0_PATH;
                }
                break;

            // -----------------------------------------------------------------
            // SHOOT 0
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
            // CYCLE 1 — row intake
            // -----------------------------------------------------------------
            case INTAKEPOS1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startGateIntakePath(paths.INTAKE1, T_INTAKE1);
                    currentState = AutoState.INTAKE1;
                }
                break;

            case INTAKE1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
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
                    startIntakePath(paths.GATEOPEN, T_GATEOPEN);
                    currentState = AutoState.GATEOPEN;
                }
                break;

            // -----------------------------------------------------------------
            // GATE CYCLE 1 — approach (99%) → 100ms delay → slow push → 1000ms dwell → shoot2
            // -----------------------------------------------------------------
            case GATEOPEN:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenPathDone()) {
                    // Stop follower at 99%, start 100ms delay
                    follower.breakFollowing();
                    gateOpenTimer.reset();
                    currentState = AutoState.GATEOPEN_DELAY;
                }
                break;

            case GATEOPEN_DELAY:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DELAY_MS) {
                    startGateIntakePath(paths.GATEINTAKE, T_GATEINTAKE);
                    currentState = AutoState.GATEINTAKE;
                }
                break;

            case GATEINTAKE:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateWaitTimer.reset();
                    currentState = AutoState.GATEINTAKE_DWELL;
                }
                break;

            case GATEINTAKE_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateWaitTimer.milliseconds() >= GATE_INTAKE_DWELL_MS) {
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
                    startIntakePath(paths.GATEINTAKE2, T_GATEINTAKE2);
                    currentState = AutoState.GATEINTAKE2;
                }
                break;

            // -----------------------------------------------------------------
            // GATE CYCLE 2 — re-approach (99%) → 100ms delay → slow push → 1000ms dwell → shoot3
            // -----------------------------------------------------------------
            case GATEINTAKE2:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenPathDone()) {
                    follower.breakFollowing();
                    gateOpenTimer.reset();
                    currentState = AutoState.GATEINTAKE2_DELAY;
                }
                break;

            case GATEINTAKE2_DELAY:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateOpenTimer.milliseconds() >= GATE_OPEN_DELAY_MS) {
                    startGateIntakePath(paths.GATEINTAKE, T_GATEINTAKE);
                    currentState = AutoState.GATEINTAKE2_PUSH;
                }
                break;

            case GATEINTAKE2_PUSH:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    follower.breakFollowing();
                    gateWaitTimer.reset();
                    currentState = AutoState.GATEINTAKE2_DWELL;
                }
                break;

            case GATEINTAKE2_DWELL:
                intake.setMode(Intake.Mode.INTAKE);
                if (gateWaitTimer.milliseconds() >= GATE_INTAKE_DWELL_MS) {
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
                    setPreShootRPM(SHOOT_4_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.INTAKE2, T_INTAKE2);
                    currentState = AutoState.INTAKE2;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 4 — row intake
            // -----------------------------------------------------------------
            case INTAKE2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startShootPath(paths.SHOOT4, T_SHOOT4);
                    currentState = AutoState.SHOOT4_PATH;
                }
                break;

            case SHOOT4_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) {
                    spinupTimer.reset();
                    currentState = AutoState.SHOOT4_SPINUP;
                }
                break;

            case SHOOT4_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT4;
                }
                break;

            case SHOOT4:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.PATH12, T_PATH12);
                    currentState = AutoState.PATH12;
                }
                break;

            // -----------------------------------------------------------------
            // PARK / LEAVE
            // -----------------------------------------------------------------
            case PATH12:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // =====================================================================
    //  PATHS  (Blue-side coordinates — doc 10)
    // =====================================================================
    public static class Paths {
        public PathChain SHOOT0;
        public PathChain INTAKEPOS1;
        public PathChain INTAKE1;
        public PathChain SHOOT1;
        public PathChain GATEOPEN;
        public PathChain GATEINTAKE;   // slow down-push (used for both gate cycles)
        public PathChain SHOOT2;
        public PathChain GATEINTAKE2;  // re-approach path (was second GATEINTAKE in original)
        public PathChain SHOOT3;
        public PathChain INTAKE2;
        public PathChain SHOOT4;
        public PathChain PATH12;

        public Paths(Follower follower) {

            // (26.316,128.947,144°) → (57.105,77.211,130°)
            SHOOT0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(26.316, 128.947),
                            new Pose(58.105,  80.211)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(133))
                    .build();

            // (57.105,77.211,130°) → (49,60.105,180°)
            INTAKEPOS1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.105, 77.211),
                            new Pose(49.000, 60.105)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                    .build();

            // (49,60.105) → (18.053,59.895) — tangent heading
            INTAKE1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(49.000, 60.105),
                            new Pose(15.053, 59.895)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // (18.053,59.895,180°) → curve → (56.947,77.211,130°)
            SHOOT1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(18.053, 59.895),
                            new Pose(49.026, 60.026),
                            new Pose(56.947, 77.211)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();

            // (56.947,77.211,130°) → curve → (13,59.474,155°)  — approach to gate
            GATEOPEN = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.947, 77.211),
                            new Pose(42.026, 63.921),
                            new Pose(13.000, 59.474)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(155))
                    .build();

            // (13,59.474,155°) → (13,52.842,120°)  — slow push down through gate
            GATEINTAKE = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.000, 59.474),
                            new Pose(13.000, 52.842)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(120))
                    .build();

            // (13,52.842,120°) → curve → (57.263,77.368,130°)
            SHOOT2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(13.000, 52.842),
                            new Pose(42.079, 60.789),
                            new Pose(57.263, 77.368)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(133))
                    .build();

            // (57.263,77.368,130°) → (13,59.053,155°)  — re-approach (second gate pass)
            GATEINTAKE2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.263, 77.368),
                            new Pose(13.000, 58.053)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(155))
                    .build();

            // (13,59.053,155°) → curve → (57.158,77.579,130°)
            SHOOT3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(13.000, 59.053),
                            new Pose(41.974, 63.421),
                            new Pose(57.158, 77.579)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(133))
                    .build();

            // (57.158,77.579) → curve → (18.632,83.947) — tangent heading
            INTAKE2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(57.158, 77.579),
                            new Pose(54.053, 82.368),
                            new Pose(46.474, 81.842),
                            new Pose(15.632, 81.947)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            // (18.632,83.947,180°) → (57.158,77.211,130°)
            SHOOT4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.632, 83.947),
                            new Pose(57.158, 77.211)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();

            // (57.158,77.211,130°) → (52,73,130°)
            PATH12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(57.158, 77.211),
                            new Pose(52.000, 73.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(133))
                    .build();
        }
    }
}