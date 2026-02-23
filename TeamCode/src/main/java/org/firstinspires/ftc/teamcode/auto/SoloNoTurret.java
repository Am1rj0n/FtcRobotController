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

@Autonomous(name = "Solo No Turret Blue", group = "Autonomous")
@Configurable
public class SoloNoTurret extends OpMode {

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
    private static final double SHOOT_0_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_1_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_2_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_3_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_4_RPM = 3400.0; // TODO: tune

    // ==================== TIMING ====================
    private static final double SHOOT_0_DURATION = 1.5; // 3 preload balls - tune
    private static final double SHOOT_DURATION   = 1.5; // intaked balls - tune

    private enum AutoState {
        SHOOT_0_PATH,  SHOOT_0,
        INTAKE_1,      SHOOT_1_POS,  SHOOT_1,

        INTAKEPOS, INTAKEPOS_WAIT, INTAKE_2,
        SHOOT_2_POS,  SHOOT_2,
        INTAKE_3,      SHOOT_3_POS,  SHOOT_3,
        INTAKE_4,      SHOOT_4_POS,  SHOOT_4,
        DONE
    }
    private AutoState currentState = AutoState.SHOOT_0_PATH;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34.000, 136.000, Math.toRadians(180)));

        intake    = new Intake(hardwareMap);
        shooter   = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap, IS_RED);
        turret    = new Turret(hardwareMap, limelight, IS_RED);
        turret.setMode(Turret.Mode.ODOMETRY);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Solo No Turret Blue - Ready");
        panelsTelemetry.debug("Turret", "ODOMETRY tracking");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
        limelight.start();

        follower.followPath(paths.Shoot0, true);
        currentState = AutoState.SHOOT_0_PATH;
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

            // ===== SHOOT 0: drive to shoot spot, shoot 3 preload balls =====
            case SHOOT_0_PATH:
                intake.setMode(Intake.Mode.OFF);
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    shooter.setTargetRPM(SHOOT_0_RPM);
                    currentState = AutoState.SHOOT_0;
                }
                break;

            case SHOOT_0:
                if (shootTimer.seconds() >= SHOOT_0_DURATION) {
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
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.shoot1, true);
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
                    follower.followPath(paths.Intakepos, true);
                    currentState = AutoState.INTAKEPOS;
                }
                break;

            // ===== CYCLE 2: gate =====

            case INTAKEPOS:
                intake.setMode(Intake.Mode.OFF);
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.INTAKE);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKEPOS_WAIT;
                }
                break;

            case INTAKEPOS_WAIT:
                intake.setMode(Intake.Mode.INTAKE); // keep intaking during wait
                if (shootTimer.seconds() >= 0.5) { // tune this delay in seconds
                    follower.followPath(paths.intake2, true);
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.shoot2, true);
                    currentState = AutoState.SHOOT_2_POS;
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
                    follower.followPath(paths.intake3, true);
                    currentState = AutoState.INTAKE_3;
                }
                break;

            // ===== CYCLE 3 =====
            case INTAKE_3:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.shoot3, true);
                    currentState = AutoState.SHOOT_3_POS;
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
                    follower.followPath(paths.intake4, true);
                    currentState = AutoState.INTAKE_4;
                }
                break;

            // ===== CYCLE 4 =====
            case INTAKE_4:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.shoot4, true);
                    currentState = AutoState.SHOOT_4_POS;
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

    // ==================== PATHS (Doc 13) ====================
    public static class Paths {
        public PathChain Shoot0, Intake1, shoot1, Intakepos, intake2, shoot2,
                intake3, shoot3, intake4, shoot4;

        public Paths(Follower follower) {
            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.000, 136.000),
                            new Pose(55.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(55.000, 90.000),
                            new Pose(48.015, 58.500),
                            new Pose(37.883, 55.526),
                            new Pose(35.474, 63.316),
                            new Pose(23.474, 59.316)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23.474, 59.316),
                            new Pose(55.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intakepos = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 90.000),
                            new Pose(13.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(156))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.000, 60.000),
                            new Pose(11.579, 54.316)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(156), Math.toRadians(120))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.579, 54.316),
                            new Pose(55.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(55.000, 90.000),
                            new Pose(38.553, 81.763),
                            new Pose(39.921, 86.237),
                            new Pose(24.842, 84.211)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.842, 84.211),
                            new Pose(55.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(55.000, 90.000),
                            new Pose(60.895, 30.579),
                            new Pose(33.789, 30.947),
                            new Pose(32.158, 38.474),
                            new Pose(24.737, 35.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.737, 35.895),
                            new Pose(56.316, 107.368)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}