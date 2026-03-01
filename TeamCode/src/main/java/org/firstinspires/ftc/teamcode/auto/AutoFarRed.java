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

@Autonomous(name = "Auto Far RED", group = "Autonomous")
@Configurable
public class AutoFarRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Paths            paths;

    private Intake  intake;
    private Shooter shooter;
    private Servo   turretServo;

    private final ElapsedTime shootTimer  = new ElapsedTime();
    private final ElapsedTime pathTimer   = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();

    private static final double FIXED_TURRET_POSITION = 0.5;

    private static final double SHOOT_0_RPM  = 4000.0;
    private static final double SHOOT_1_RPM  = 4000.0;
    private static final double SHOOT_2_RPM  = 4000.0;
    private static final double SHOOT_2B_RPM = 4000.0;
    private static final double SHOOT_3_RPM  = 2800.0;
    private static final double SHOOT_4_RPM  = 2800.0;

    private static final double INITIAL_SPINUP_DURATION = 1.2;
    private static final double SPINUP_DURATION         = 1.00;
    private static final double SHOOT_DURATION          = 2.0;

    private static final double T_SHOOT0       = 3.0;
    private static final double T_INTAKE1POS   = 3.0;
    private static final double T_INTAKE1      = 3.0;
    private static final double T_SHOOT1       = 3.0;
    private static final double T_INTAKEPOS2   = 2.0;
    private static final double T_INTAKE2      = 1.5;
    private static final double T_INTAKE2_REV  = 1.5;
    private static final double T_INTAKE2_FWD  = 1.5;
    private static final double T_SHOOT2       = 3.0;
    private static final double T_INTAKEPOS2B  = 2.0;
    private static final double T_INTAKE2B     = 1.5;
    private static final double T_INTAKE2B_REV = 1.5;
    private static final double T_INTAKE2B_FWD = 1.5;
    private static final double T_SHOOT2B      = 3.0;
    private static final double T_PARK         = 3.0;

    private static final double INTAKE_PATH_SPEED = 0.75;

    private double currentPathTimeout = 4.0;

    private enum AutoState {
        INITIAL_SPINUP,

        SHOOT_0_PATH, SHOOT_0_SPINUP, SHOOT_0,

        INTAKE1POS, INTAKE1,
        SHOOT1_PATH, SHOOT1_SPINUP, SHOOT1,

        INTAKEPOS2, INTAKE2, INTAKE2_REV, INTAKE2_FWD,
        SHOOT2_PATH, SHOOT2_SPINUP, SHOOT2,

        INTAKEPOS2B, INTAKE2B, INTAKE2B_REV, INTAKE2B_FWD,
        SHOOT2B_PATH, SHOOT2B_SPINUP, SHOOT2B,

        PARK,
        DONE
    }

    private AutoState currentState = AutoState.INITIAL_SPINUP;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Mirror of (56,8,90°) → x=144-56=88
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(FIXED_TURRET_POSITION);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Auto Far Red - Ready");
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

    // ── Helpers ──────────────────────────────────────────────────────────────

    private void startIntakePath(PathChain path, double timeout) {
        follower.setMaxPower(INTAKE_PATH_SPEED);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private void startShootPath(PathChain path, double timeout) {
        follower.setMaxPower(0.7);
        follower.followPath(path, true);
        pathTimer.reset();
        currentPathTimeout = timeout;
    }

    private boolean pathDone() {
        return follower.getCurrentTValue() >= 0.95
                || !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    private boolean shootPathDone() {
        return !follower.isBusy()
                || pathTimer.seconds() >= currentPathTimeout;
    }

    private void setPreShootRPM(double rpm) {
        shooter.setTargetRPM(rpm);
    }

    // ── State machine ────────────────────────────────────────────────────────

    private void autonomousUpdate() {
        switch (currentState) {

            case INITIAL_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= INITIAL_SPINUP_DURATION) {
                    startShootPath(paths.shoot0, T_SHOOT0);
                    currentState = AutoState.SHOOT_0_PATH;
                }
                break;

            case SHOOT_0_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); currentState = AutoState.SHOOT_0_SPINUP; }
                break;
            case SHOOT_0_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset(); intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_0;
                }
                break;
            case SHOOT_0:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    setPreShootRPM(SHOOT_1_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.intake1pos, T_INTAKE1POS);
                    currentState = AutoState.INTAKE1POS;
                }
                break;

            case INTAKE1POS:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake1, T_INTAKE1); currentState = AutoState.INTAKE1; }
                break;
            case INTAKE1:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startShootPath(paths.shoot1, T_SHOOT1); currentState = AutoState.SHOOT1_PATH; }
                break;
            case SHOOT1_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); currentState = AutoState.SHOOT1_SPINUP; }
                break;
            case SHOOT1_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset(); intake.setMode(Intake.Mode.SHOOT);
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

            case INTAKEPOS2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2, T_INTAKE2); currentState = AutoState.INTAKE2; }
                break;
            case INTAKE2:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2retry, T_INTAKE2_REV); currentState = AutoState.INTAKE2_REV; }
                break;
            case INTAKE2_REV:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2done, T_INTAKE2_FWD); currentState = AutoState.INTAKE2_FWD; }
                break;
            case INTAKE2_FWD:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startShootPath(paths.shoot2, T_SHOOT2); currentState = AutoState.SHOOT2_PATH; }
                break;
            case SHOOT2_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); currentState = AutoState.SHOOT2_SPINUP; }
                break;
            case SHOOT2_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset(); intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT2;
                }
                break;
            case SHOOT2:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    setPreShootRPM(SHOOT_2B_RPM);
                    intake.setMode(Intake.Mode.INTAKE);
                    startIntakePath(paths.intakepos2b, T_INTAKEPOS2B);
                    currentState = AutoState.INTAKEPOS2B;
                }
                break;

            case INTAKEPOS2B:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2b, T_INTAKE2B); currentState = AutoState.INTAKE2B; }
                break;
            case INTAKE2B:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2retry_b, T_INTAKE2B_REV); currentState = AutoState.INTAKE2B_REV; }
                break;
            case INTAKE2B_REV:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startIntakePath(paths.intake2done_b, T_INTAKE2B_FWD); currentState = AutoState.INTAKE2B_FWD; }
                break;
            case INTAKE2B_FWD:
                intake.setMode(Intake.Mode.INTAKE);
                if (pathDone()) { startShootPath(paths.shoot2b, T_SHOOT2B); currentState = AutoState.SHOOT2B_PATH; }
                break;
            case SHOOT2B_PATH:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { spinupTimer.reset(); currentState = AutoState.SHOOT2B_SPINUP; }
                break;
            case SHOOT2B_SPINUP:
                intake.setMode(Intake.Mode.INTAKE);
                if (spinupTimer.seconds() >= SPINUP_DURATION) {
                    shootTimer.reset(); intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT2B;
                }
                break;
            case SHOOT2B:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.INTAKE);
                    startShootPath(paths.park, T_PARK);
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootPathDone()) { currentState = AutoState.DONE; }
                break;

            case DONE:
                intake.setMode(Intake.Mode.INTAKE);
                break;
        }
    }

    // ── Paths (Red mirror: x_red = 144 - x_blue, heading_red = 180° - heading_blue) ──

    public static class Paths {
        public PathChain shoot0;
        public PathChain intake1pos;
        public PathChain intake1;
        public PathChain shoot1;
        public PathChain intakepos2;
        public PathChain intake2;
        public PathChain intake2retry;
        public PathChain intake2done;
        public PathChain shoot2;
        public PathChain intakepos2b;
        public PathChain intake2b;
        public PathChain intake2retry_b;
        public PathChain intake2done_b;
        public PathChain shoot2b;
        public PathChain park;

        public Paths(Follower follower) {

            // Blue start: (56,8,90°)   → Red: (88,8,90°)
            // Blue end:   (55.737,10.895,111°) → Red: (88.263,10.895,69°)
            shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.000,  8.000),
                            new Pose(88.263, 10.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))
                    .build();

            // Blue: (55.737,10.895,111°) → (44.368,35.053,180°)
            // Red:  (88.263,10.895, 69°) → (99.632,35.053,  0°)
            intake1pos = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.263, 10.895),
                            new Pose(99.632, 35.053)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                    .build();

            // Blue: (44.368,35.053,180°) → (9.947,34.895,180°)
            // Red:  (99.632,35.053,  0°) → (134.053,34.895, 0°)
            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(99.632,  35.053),
                            new Pose(134.053, 34.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue: (9.947,34.895,180°) → (55.895,10.895,111°)
            // Red:  (134.053,34.895,0°) → (88.105,10.895, 69°)
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.053, 34.895),
                            new Pose(88.105,  10.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                    .build();

            // ── Cycle 2 ──────────────────────────────────────────────────────

            // Blue curve: (55.895,10.895) → (43.316,20.395) → (20,8)   111°→180°
            // Red  curve: (88.105,10.895) → (100.684,20.395) → (124,8)  69°→0°
            intakepos2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88.105,  10.895),
                            new Pose(100.684, 20.395),
                            new Pose(124.000,  8.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                    .build();

            // Blue: (20,8) → (8,8)   180°→180°
            // Red:  (124,8) → (136,8)  0°→0°
            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.000, 8.000),
                            new Pose(136.000, 8.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue: (8,8) → (19.553,8.553)   180°→180°
            // Red:  (136,8) → (124.447,8.553)  0°→0°
            intake2retry = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(136.000, 8.000),
                            new Pose(124.447, 8.553)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue: (19.553,8.553) → (8.276,7.882)   180°→180°
            // Red:  (124.447,8.553) → (135.724,7.882)  0°→0°
            intake2done = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.447, 8.553),
                            new Pose(135.724, 7.882)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // Blue curve: (8.276,7.882) → (33.974,18.395) → (56,11)   180°→111°
            // Red  curve: (135.724,7.882) → (110.026,18.395) → (88,11)  0°→69°
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(135.724,  7.882),
                            new Pose(110.026, 18.395),
                            new Pose(88.000,  11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                    .build();

            // ── Cycle 2B (repeat) ─────────────────────────────────────────────

            intakepos2b = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(88.105,  10.895),
                            new Pose(100.684, 20.395),
                            new Pose(124.000,  8.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(0))
                    .build();

            intake2b = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.000, 8.000),
                            new Pose(136.000, 8.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2retry_b = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(136.000, 8.000),
                            new Pose(124.447, 8.553)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2done_b = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(124.447, 8.553),
                            new Pose(135.724, 7.882)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2b = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(135.724,  7.882),
                            new Pose(110.026, 18.395),
                            new Pose(88.000,  11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(69))
                    .build();

            // ── Park ──────────────────────────────────────────────────────────

            // Blue: (56,11,111°) → (36.316,10.842,90°)
            // Red:  (88,11, 69°) → (107.684,10.842,90°)
            park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(88.000,   11.000),
                            new Pose(107.684,  10.842)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(90))
                    .build();
        }
    }
}