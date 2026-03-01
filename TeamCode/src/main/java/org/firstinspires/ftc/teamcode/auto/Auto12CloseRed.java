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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Auto 12 Close Red", group = "Autonomous")
@Configurable
public class Auto12CloseRed extends OpMode {

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
    private final ElapsedTime shootTimer  = new ElapsedTime();
    private final ElapsedTime pathTimer   = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();

    // =====================================================================
    //  TURRET
    // =====================================================================
    private static final double FIXED_TURRET_POSITION = 0.5; // TODO: tune for red

    // =====================================================================
    //  SHOOTER RPMs  — tune each independently
    // =====================================================================
    private static final double SHOOT_0_RPM = 3000.0;
    private static final double SHOOT_1_RPM = 3000.0;
    private static final double SHOOT_2_RPM = 3000.0;
    private static final double SHOOT_3_RPM = 3000.0;

    // =====================================================================
    //  TIMING CONSTANTS
    // =====================================================================
    private static final double INITIAL_SPINUP_DURATION = 1.0;
    private static final double SPINUP_DURATION         = 0.4;
    private static final double SHOOT_DURATION          = 1.7;

    // =====================================================================
    //  PER-PATH TIMEOUTS  — tune each independently (seconds)
    // =====================================================================
    private static final double T_SHOOT0      = 4.0;
    private static final double T_INTAKEPOS1  = 2.0;
    private static final double T_INTAKE1     = 3.0;
    private static final double T_GATEPOS     = 2.0;
    private static final double T_GATEOPEN    = 1.0;
    private static final double T_SHOOT1      = 3.0;
    private static final double T_INTAKEPOS2  = 3.0;
    private static final double T_INTAKE2     = 3.0;
    private static final double T_SHOOT2      = 3.0;
    private static final double T_INTAKEPOS3  = 3.0;
    private static final double T_INTAKE3     = 3.0;
    private static final double T_SHOOT3      = 3.0;
    private static final double T_LEAVE       = 3.0;

    // =====================================================================
    //  PATH SPEED
    // =====================================================================
    private static final double INTAKE_PATH_SPEED = 0.9;

    // =====================================================================
    //  INTERNAL STATE
    // =====================================================================
    private double currentPathTimeout = 4.0;

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
        GATEPOS,
        GATEOPEN,
        SHOOT1_PATH,
        SHOOT1_SPINUP,
        SHOOT1,

        INTAKEPOS2,
        INTAKE2,
        SHOOT2_PATH,
        SHOOT2_SPINUP,
        SHOOT2,

        INTAKEPOS3,
        INTAKE3,
        SHOOT3_PATH,
        SHOOT3_SPINUP,
        SHOOT3,

        LEAVE,
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
        follower.setStartingPose(new Pose(109.000, 137.000, Math.toRadians(0)));

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(FIXED_TURRET_POSITION);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Auto 12 Close Red - Ready");
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

        panelsTelemetry.debug("State",      currentState.name());
        panelsTelemetry.debug("T Value",    follower.getCurrentTValue());
        panelsTelemetry.debug("Busy",       follower.isBusy());
        panelsTelemetry.debug("Path Timer", String.format("%.2f / %.1f", pathTimer.seconds(), currentPathTimeout));
        panelsTelemetry.debug("Spinup",     String.format("%.2f / %.1f", spinupTimer.seconds(), SPINUP_DURATION));
        panelsTelemetry.debug("RPM Target", shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",   shooter.getReadRPM());
        panelsTelemetry.debug("At Speed",   shooter.isAtSpeed());
        panelsTelemetry.debug("Intake",     intake.getCurrentMode().toString());
        panelsTelemetry.debug("X",          follower.getPose().getX());
        panelsTelemetry.debug("Y",          follower.getPose().getY());
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

    private void startShootPath(PathChain path, double timeout) {
        follower.setMaxPower(1.0);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    /** Intake/transit: 95% T-value OR follower done OR timeout. */
    private boolean pathDone() {
        return follower.getCurrentTValue() >= 0.95
                || !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    /** Shoot positioning: follower fully done OR timeout. */
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
                    startShootPath(paths.shoot0, T_SHOOT0);
                    currentState = AutoState.SHOOT_0_PATH;
                }
                break;

            // -----------------------------------------------------------------
            // SHOOT 0  — preload
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
                    startIntakePath(paths.intakepos1, T_INTAKEPOS1);
                    currentState = AutoState.INTAKEPOS1;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 1  — intake at high row then open gate
            // -----------------------------------------------------------------
            case INTAKEPOS1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.intake1, T_INTAKE1);
                    currentState = AutoState.INTAKE1;
                }
                break;

            case INTAKE1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.gatepos, T_GATEPOS);
                    currentState = AutoState.GATEPOS;
                }
                break;

            case GATEPOS:       // back to gate approach position
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.gateopen, T_GATEOPEN);
                    currentState = AutoState.GATEOPEN;
                }
                break;

            case GATEOPEN:      // push gate open
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startShootPath(paths.shoot1, T_SHOOT1);
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
                    startIntakePath(paths.intakepos2, T_INTAKEPOS2);
                    currentState = AutoState.INTAKEPOS2;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 2
            // -----------------------------------------------------------------
            case INTAKEPOS2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.intake2, T_INTAKE2);
                    currentState = AutoState.INTAKE2;
                }
                break;

            case INTAKE2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startShootPath(paths.shoot2, T_SHOOT2);
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
                    startIntakePath(paths.intakepos3, T_INTAKEPOS3);
                    currentState = AutoState.INTAKEPOS3;
                }
                break;

            // -----------------------------------------------------------------
            // CYCLE 3
            // -----------------------------------------------------------------
            case INTAKEPOS3:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startIntakePath(paths.intake3, T_INTAKE3);
                    currentState = AutoState.INTAKE3;
                }
                break;

            case INTAKE3:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) {
                    startShootPath(paths.shoot3, T_SHOOT3);
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
                    startIntakePath(paths.leave, T_LEAVE);
                    currentState = AutoState.LEAVE;
                }
                break;

            // -----------------------------------------------------------------
            // LEAVE
            // -----------------------------------------------------------------
            case LEAVE:
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
    //  PATHS  (Red-side coordinates — doc 9)
    // =====================================================================
    public static class Paths {
        public PathChain shoot0;
        public PathChain intakepos1;
        public PathChain intake1;
        public PathChain gatepos;
        public PathChain gateopen;
        public PathChain shoot1;
        public PathChain intakepos2;
        public PathChain intake2;
        public PathChain shoot2;
        public PathChain intakepos3;
        public PathChain intake3;
        public PathChain shoot3;
        public PathChain leave;

        public Paths(Follower follower) {

            // (109,137,0°) → (89,84,50°)
            shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(109.000, 137.000),
                            new Pose(89.000,   84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            // (89,84,50°) → (98.474,84.158,0°)
            intakepos1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.000,  84.000),
                            new Pose(98.474,  84.158)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            // (98.474,84.158,0°) → (124.421,84.421,0°)
            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(98.474,  84.158),
                            new Pose(124.421, 84.421)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // (124.421,84.421,0°) → (118.553,74.737,0°)
            gatepos = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.421, 84.421),
                            new Pose(118.553, 78.737)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // (118.553,74.737,0°) → (128.329,74.711,0°)
            gateopen = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(118.553, 78.737),
                            new Pose(128.329, 78.711)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // (128.329,74.711,0°) → (89,84,50°)
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(128.329, 78.711),
                            new Pose(89.000,   84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            // (89,84,50°) → (95.842,59.263,0°)
            intakepos2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.000,  84.000),
                            new Pose(95.842,  59.263)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            // (95.842,59.263,0°) → (127.105,59.842,0°)
            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(95.842,  59.263),
                            new Pose(133.105, 59.842)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // (127.105,59.842,0°) → (89,84.105,50°)
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(133.105, 59.842),
                            new Pose(89.000,   84.105)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            // (89,84.105,50°) → (96.632,35.579,0°)
            intakepos3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(89.000,  84.105),
                            new Pose(96.632,  35.579)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            // (96.632,35.579,0°) → (125.737,35.947,0°)
            intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(96.632,  35.579),
                            new Pose(132.737, 35.947)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            // (125.737,35.947,0°) → (88.895,84.421,50°)
            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(132.737, 35.947),
                            new Pose(88.895,  84.421)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            // (88.895,84.421,50°) → (119.105,68.526,0°)
            leave = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.895,  84.421),
                            new Pose(119.105, 68.526)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();
        }
    }
}