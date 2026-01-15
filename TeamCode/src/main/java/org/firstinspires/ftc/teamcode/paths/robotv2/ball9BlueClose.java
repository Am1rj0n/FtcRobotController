package org.firstinspires.ftc.teamcode.paths.robotv2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

// Note: Replace these with your actual subsystem imports
import org.firstinspires.ftc.teamcode.subfilesV2.ShooterSubsystem;

@Autonomous(name = "9 Ball - Blue", group = "Autonomous")
@Configurable
public class ball9BlueClose extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private ElapsedTime shootTimer = new ElapsedTime();

    // Hardware
    private DcMotor intakeMotor, transferMotor;
    private ShooterSubsystem shooter;

    // Constants
    private static final double SHOOT_DURATION = 2.0;
    private static final double INTAKE_POWER = -1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.15;
    private static final double SHOOT_INTAKE_POWER = -0.55;
    private static final double SHOOT_TRANSFER_POWER = 0.5;

    private enum AutoState {
        DRIVE_TO_SHOOT_0, SHOOT_0,
        DRIVE_TO_INTAKE_1, INTAKE_1, DRIVE_TO_SHOOT_1, SHOOT_1,
        DRIVE_TO_INTAKE_2, INTAKE_2, DRIVE_TO_SHOOT_2, SHOOT_2,
        IDLE
    }

    private AutoState currentState = AutoState.DRIVE_TO_SHOOT_0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Start pose must match the start of Shoot0
        follower.setStartingPose(new Pose(15.000, 110.000, Math.toRadians(90)));

        paths = new Paths(follower);

        // Hardware Mapping
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = new ShooterSubsystem(hardwareMap, false);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.enable();
        shooter.setFarMode();
        follower.followPath(paths.Shoot0);
        currentState = AutoState.DRIVE_TO_SHOOT_0;
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case DRIVE_TO_SHOOT_0:
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_0;
                }
                break;

            case SHOOT_0:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.IntakePos1);
                    currentState = AutoState.DRIVE_TO_INTAKE_1;
                }
                break;

            case DRIVE_TO_INTAKE_1:
                runIntakeMode(); // Intake turns on while moving
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake1, true);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    currentState = AutoState.DRIVE_TO_SHOOT_1;
                }
                break;

            case DRIVE_TO_SHOOT_1:
                runIntakeMode(); // Keep intaking until at position
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.Intakepos2);
                    currentState = AutoState.DRIVE_TO_INTAKE_2;
                }
                break;

            case DRIVE_TO_INTAKE_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake2, true);
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootpos);
                    currentState = AutoState.DRIVE_TO_SHOOT_2;
                }
                break;

            case DRIVE_TO_SHOOT_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_2;
                }
                break;

            case SHOOT_2:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                stopIntake();
                shooter.disable();
                break;
        }
    }

    private void runIntakeMode() {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(INTAKE_TRANSFER_POWER);
    }

    private void runShootMode() {
        intakeMotor.setPower(SHOOT_INTAKE_POWER);
        transferMotor.setPower(SHOOT_TRANSFER_POWER);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    public static class Paths {
        public PathChain Shoot0, IntakePos1, Intake1, shoot1, Intakepos2, intake2, shootpos;

        public Paths(Follower follower) {
            Shoot0 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15.000, 110.000), new Pose(58.000, 88.000))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132)).build();

            IntakePos1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.000, 88.000), new Pose(50.883, 54.938))
            ).setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0)).build();

            Intake1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(50.883, 54.938), new Pose(25.000, 54.855))
            ).setTangentHeadingInterpolation().setReversed().build();

            shoot1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(25.000, 54.855), new Pose(58.000, 88.000))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132)).build();

            Intakepos2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.000, 88.000), new Pose(50.483, 79.586))
            ).setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0)).build();

            intake2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(50.483, 79.586), new Pose(20.966, 79.062))
            ).setTangentHeadingInterpolation().setReversed().build();

            shootpos = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(20.966, 79.062), new Pose(58.166, 88.000))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132)).build();
        }
    }
}