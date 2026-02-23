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


@Autonomous(name = "Auto Solo Blue", group = "Autonomous")
@Configurable
public class AutoSolo extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private Intake intake;
    private Shooter shooter;
    private Servo turretServo; // Fixed - no tracking

    private final ElapsedTime shootTimer = new ElapsedTime();

    // ==================== TUNE THESE RPMs ====================
    private static final double SHOOT_0_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_1_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_2_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_3_RPM = 3300.0; // TODO: tune
    private static final double SHOOT_4_RPM = 3000.0; // TODO: tune

    // Fixed turret position - tune to aim at goal from shooting spots
    private static final double FIXED_TURRET_POSITION = 0.5; // TODO: tune

    // ==================== TIMING ====================
    private static final double SHOOT_0_DURATION = 1.5; // 3 preload balls - tune
    private static final double SHOOT_DURATION   = 1.5; // intaked balls - tune

    private enum AutoState {
        SHOOT_0_PATH,  SHOOT_0,
        INTAKEPOS_1,   INTAKE_1,  SHOOT_1_POS,  SHOOT_1,
        INTAKE_2_POS, INTAKE_2_POS_WAIT ,INTAKE_2,  SHOOT_2_POS,  SHOOT_2,
        INTAKEPOS_3,   INTAKE_3,  SHOOT_3_POS,  SHOOT_3,
        INTAKE_4,      SHOOT_4_POS, SHOOT_4,
        DONE
    }
    private AutoState currentState = AutoState.SHOOT_0_PATH;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34.000, 136.000, Math.toRadians(180)));

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(FIXED_TURRET_POSITION);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Auto Solo Blue - Ready");
        panelsTelemetry.debug("Turret", "FIXED at " + FIXED_TURRET_POSITION);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.spin();
        shooter.setTargetRPM(SHOOT_0_RPM);
        turretServo.setPosition(FIXED_TURRET_POSITION);

        follower.followPath(paths.Shoot0, true);
        currentState = AutoState.SHOOT_0_PATH;
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
        panelsTelemetry.debug("RPM Target", shooter.getTargetRPM());
        panelsTelemetry.debug("RPM Read",   shooter.getReadRPM());
        panelsTelemetry.debug("At Speed",   shooter.isAtSpeed());
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
                    follower.followPath(paths.Intakepos1, true);
                    currentState = AutoState.INTAKEPOS_1;
                }
                break;

            // ===== CYCLE 1 =====
            case INTAKEPOS_1:
                intake.setMode(Intake.Mode.OFF);
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.INTAKE);
                }
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake1, true);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
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
                    follower.followPath(paths.intake2pos, true);
                    currentState = AutoState.INTAKE_2_POS;
                }
                break;

            // ===== CYCLE 2 =====
            case INTAKE_2_POS:
                intake.setMode(Intake.Mode.OFF);
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.INTAKE);
                }
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_2_POS_WAIT;
                }
                break;

            case INTAKE_2_POS_WAIT:
                intake.setMode(Intake.Mode.INTAKE);
                if (shootTimer.seconds() >= 0.5) {
                    follower.followPath(paths.intake2, true);
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Shoot2, true);
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
                    follower.followPath(paths.Intakepos3, true);
                    currentState = AutoState.INTAKEPOS_3;
                }
                break;

            // ===== CYCLE 3 =====
            case INTAKEPOS_3:
                intake.setMode(Intake.Mode.OFF);
                if (!follower.isBusy() || follower.getCurrentTValue() >= 0.95) {
                    intake.setMode(Intake.Mode.INTAKE);
                }
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake3, true);
                    currentState = AutoState.INTAKE_3;
                }
                break;


            case INTAKE_3:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Shoot3, true);
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
                    follower.followPath(paths.intake4, true);
                    currentState = AutoState.INTAKE_4;
                }
                break;

            // ===== CYCLE 4: intake4 → Path12 final position → shoot =====
            case INTAKE_4:
                intake.setMode(Intake.Mode.INTAKE);
                if (!follower.isBusy()) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.setTargetRPM(SHOOT_4_RPM);
                    follower.followPath(paths.Path12, true);
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
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    // ==================== PATHS (Doc 12) ====================
    public static class Paths {
        public PathChain Shoot0, Intakepos1, Intake1, Shoot1,
                intake2pos, intake2, Shoot2,
                Intakepos3, Intake3, Shoot3,
                intake4, Path12;

        public Paths(Follower follower) {
            Shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(34.000, 136.000),
                            new Pose(50.789, 92.684)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                    .build();

            Intakepos1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(50.789, 92.684),
                            new Pose(43.474, 60.474)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                    .build();

            Intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(43.474, 60.474),
                            new Pose(22.632, 59.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(22.632, 59.895),
                            new Pose(56.000, 80.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129.5))
                    .build();

            intake2pos = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 80.000),
                            new Pose(13.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(129.5), Math.toRadians(156))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.000, 60.000),
                            new Pose(11.579, 54.316)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(156), Math.toRadians(120))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.579, 54.316),
                            new Pose(56.000, 80.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(129.5))
                    .build();

            Intakepos3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 80.000),
                            new Pose(45.053, 35.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(129.5), Math.toRadians(180))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(45.053, 35.895),
                            new Pose(24.263, 36.105)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.263, 36.105),
                            new Pose(46.842, 83.895)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            intake4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(46.842, 83.895),
                            new Pose(24.105, 84.211)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.105, 84.211),
                            new Pose(61.000, 101.947)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();
        }
    }
}