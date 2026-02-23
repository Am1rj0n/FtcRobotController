package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Auto Alliance Blue - Doc 14 paths
 * - Starts at (56, 8)
 * - Turret tracks goal via ODOMETRY the entire auto
 * - Shooter on from start to end
 * - Shoot preload from starting position → 4 intake/shoot cycles → Leave
 *
 * Hardware: drive motors, Pinpoint, intake, transfer, s1, s2, turret servo, limelight
 */
@Autonomous(name = "Auto Alliance Far Blue", group = "Autonomous")
@Configurable
public class AutoAlliance extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private final ElapsedTime shootTimer = new ElapsedTime();

    private static final boolean IS_RED = false;

    // ==================== TUNE THESE RPMs ====================
    private static final double PRELOAD_RPM = 4000.0; // TODO: tune
    private static final double SHOOT_1_RPM = 4100.0; // TODO: tune
    private static final double SHOOT_2_RPM = 4100.0; // TODO: tune
    private static final double SHOOT_3_RPM = 4100.0; // TODO: tune
    private static final double SHOOT_4_RPM = 4100.0; // TODO: tune

    // ==================== TIMING ====================
    private static final double PRELOAD_SHOOT_DURATION = 1.5; // 3 preload balls - tune
    private static final double SHOOT_DURATION         = 1.5; // intaked balls - tune

    private enum AutoState {
        SHOOT_PRELOAD,
        // Enum becomes:
        INTAKE_1, INTAKE_1_WAIT, SHOOT_1_POS, SHOOT_1,
        INTAKE_2, INTAKE_2_WAIT, SHOOT_2_POS, SHOOT_2,
        INTAKE_3, INTAKE_3_WAIT, SHOOT_3_POS, SHOOT_3,
        INTAKE_4, INTAKE_4_WAIT, SHOOT_4_POS, SHOOT_4,
        LEAVE,      DONE
    }
    private AutoState currentState = AutoState.SHOOT_PRELOAD;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(180)));

        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap, IS_RED);
        turret    = new Turret(hardwareMap, limelight, IS_RED);
        turret.setMode(Turret.Mode.ODOMETRY);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Auto Alliance Blue - Ready");
        panelsTelemetry.debug("Turret", "ODOMETRY tracking");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(PRELOAD_RPM);
        limelight.start();

        shootTimer.reset();
        currentState = AutoState.SHOOT_PRELOAD;
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        turret.update(follower.getPose());

        autonomousUpdate();

        AutoToTeleTransfer.finalPose = follower.getPose();

        panelsTelemetry.debug("State",      currentState.name());
        panelsTelemetry.debug("T Value",    follower.getCurrentTValue());
        panelsTelemetry.debug("Busy",       follower.isBusy());
        panelsTelemetry.debug("RPM Target", shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",   shooter.getReadRPM());
        panelsTelemetry.debug("At Speed",   shooter.isAtSpeed());
        panelsTelemetry.debug("Turret Ang", turret.getTargetAngle());
        panelsTelemetry.debug("Aligned",    turret.isAligned());
        panelsTelemetry.debug("Intake",     intake.getCurrentMode().toString());
        panelsTelemetry.debug("X",          follower.getPose().getX());
        panelsTelemetry.debug("Y",          follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousUpdate() {
        switch (currentState) {

            // ===== PRELOAD: shoot from (56, 8) without moving =====
            case SHOOT_PRELOAD:
                intake.setMode(Intake.Mode.SHOOT);
                if (shootTimer.seconds() >= PRELOAD_SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_1_RPM);
                    follower.followPath(paths.Intake1, true);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            // ===== CYCLE 1 =====
            case INTAKE_1:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_1_WAIT;
                }
                break;

            case INTAKE_1_WAIT:
                intake.setMode(Intake.Mode.INTAKE); // still intaking during 200ms
                if (shootTimer.seconds() >= 0.2) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Shoot1, true);
                    currentState = AutoState.SHOOT_1_POS;
                }
                break;

            case SHOOT_1_POS:
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_1_RPM);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_2_RPM);
                    follower.followPath(paths.Intake2, true);
                    currentState = AutoState.INTAKE_2;
                }
                break;

            // ===== CYCLE 2 =====
            case INTAKE_2:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_2_WAIT; // was going straight to SHOOT_2_POS
                }
                break;


            case INTAKE_2_WAIT:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootTimer.seconds() >= 0.2) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Shoot2, true); // was wrongly paths.Shoot1
                    currentState = AutoState.SHOOT_2_POS;   // was wrongly SHOOT_1_POS
                }
                break;

            case SHOOT_2_POS:
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_2_RPM);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_2;
                }
                break;

            case SHOOT_2:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_3_RPM);
                    follower.followPath(paths.Intake3, true);
                    currentState = AutoState.INTAKE_3;
                }
                break;

            // ===== CYCLE 3 =====
            case INTAKE_3:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_3_WAIT;
                }
                break;

            case INTAKE_3_WAIT:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootTimer.seconds() >= 0.2) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Shoot3, true); // was wrongly paths.Shoot1
                    currentState = AutoState.SHOOT_3_POS;   // was wrongly SHOOT_1_POS
                }
                break;

            case SHOOT_3_POS:
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_3_RPM);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_4_RPM);
                    follower.followPath(paths.Intake4, true);
                    currentState = AutoState.INTAKE_4;
                }
                break;

            // ===== CYCLE 4 =====
            case INTAKE_4:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_4_WAIT;
                }
                break;

            case INTAKE_4_WAIT:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootTimer.seconds() >= 0.2) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.shoot4, true); // was wrongly paths.Shoot1
                    currentState = AutoState.SHOOT_4_POS;   // was wrongly SHOOT_1_POS
                }
                break;

            case SHOOT_4_POS:
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_4_RPM);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    currentState = AutoState.SHOOT_4;
                }
                break;

            case SHOOT_4:
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Leave, true);
                    currentState = AutoState.LEAVE;
                }
                break;

            // ===== LEAVE =====
            case LEAVE:
                if (!follower.isBusy()) {
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                intake.setMode(Intake.Mode.OFF);
                break;
        }
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        limelight.stop();
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    // ==================== PATHS (Doc 14) ====================
    public static class Paths {
        public PathChain Intake1, Shoot1, Intake2, Shoot2, Intake3, Shoot3, Intake4, shoot4, Leave;

        public Paths(Follower follower) {
            Intake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.000, 8.000),
                            new Pose(62.237, 35.158),
                            new Pose(21.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21.000, 36.000),
                            new Pose(54.000, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 11.000),
                            new Pose(9.000, 10.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000, 10.000),
                            new Pose(54.000, 11.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(54.000, 11.000),
                            new Pose(35.500, 14.895),
                            new Pose(9.842, 16.474)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.842, 16.474),
                            new Pose(54.000, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 11.000),
                            new Pose(9.737, 16.421)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.737, 16.421),
                            new Pose(54.000, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 11.000),
                            new Pose(38.474, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}