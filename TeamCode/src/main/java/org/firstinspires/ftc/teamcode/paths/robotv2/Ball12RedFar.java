package org.firstinspires.ftc.teamcode.paths.robotv2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subfilesV2.LightingSubsystem;

@Autonomous(name = "12 Ball Red Far - V3", group = "Autonomous")
@Configurable
public class Ball12RedFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // Intake/Transfer Hardware
    private DcMotor intakeMotor, transferMotor;
    private DistanceSensor intakeSensor;

    // Shooter Hardware (Controlled directly in Auto)
    private DcMotor shooterTop, shooterBottom;
    private LightingSubsystem lights;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();

    public static Pose autoEndPose = null;

    // --- SHOOTER DIRECT POWER SETTINGS ---
    private static final double TOP_MOTOR_POWER = 0.66;
    private static final double BOTTOM_MOTOR_POWER = 0.67;

    // --- FEEDER POWER CONSTANTS ---
    private static final double INTAKE_POWER = -1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.15;
    private static final double SHOOT_INTAKE_POWER = -0.55;
    private static final double SHOOT_TRANSFER_POWER = 0.5;

    // --- TUNING CONSTANTS ---
    private static final double SPIN_UP_DURATION = 1.0;
    private static final double SHOOT_DURATION = 3.0; //was 2.5
    private static final double JAM_DISTANCE_CM = 14.0;
    private static final double JAM_TIME = 0.7;

    private boolean objectDetected = false;
    private boolean jammed = false;

    private enum AutoState {
        SPIN_UP, START_TO_SHOOT, SHOOT_PRELOAD,
        INTAKE_1_POS, INTAKE_1, SHOOT_1_POS, SHOOT_1,
        INTAKE_2_POS, INTAKE_2, SHOOT_2_POS, SHOOT_2,
        INTAKE_3_POS, INTAKE_3, SHOOT_3_POS, SHOOT_3,
        PARK, IDLE
    }
    private AutoState currentState = AutoState.SPIN_UP;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        Pose bluePose = new Pose(56, 8, Math.toRadians(90));
        follower.setStartingPose(bluePose.mirror());

        paths = new Paths(follower);

        // Intake/Transfer Init
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        // Shooter Motor Init
        shooterTop = hardwareMap.get(DcMotor.class, "outtakeMotor2");
        shooterBottom = hardwareMap.get(DcMotor.class, "outtakeMotor1");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure shooter directions are correct for your build
        shooterTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set to BRAKE to stop faster when Auto ends
        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lights = new LightingSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        shootTimer.reset();
        currentState = AutoState.SPIN_UP;
    }

    @Override
    public void loop() {
        follower.update();
        updateJamDetection();
        autonomousPathUpdate();
        lights.update();

        // --- DIRECT SHOOTER CONTROL LOGIC ---
        if (currentState != AutoState.IDLE) {
            // Apply your custom powers constantly during the run
            shooterTop.setPower(TOP_MOTOR_POWER);
            shooterBottom.setPower(BOTTOM_MOTOR_POWER);
        } else {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }

        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("Shooter Top", shooterTop.getPower());
        panelsTelemetry.debug("Shooter Bottom", shooterBottom.getPower());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case SPIN_UP:
                stopIntake();
                // Waiting for the motors (already set to 0.67/0.70 in loop) to reach speed
                if (shootTimer.seconds() > SPIN_UP_DURATION) {
                    follower.followPath(paths.StartToShoot);
                    currentState = AutoState.START_TO_SHOOT;
                }
                break;

            case START_TO_SHOOT:
                stopIntake();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.Intakepos1);
                    currentState = AutoState.INTAKE_1_POS;
                }
                break;

            case INTAKE_1_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1, true);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shootpos);
                    currentState = AutoState.SHOOT_1_POS;
                }
                break;

            case SHOOT_1_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.intakepos2);
                    currentState = AutoState.INTAKE_2_POS;
                }
                break;

            case INTAKE_2_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake2, true);
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootpos2);
                    currentState = AutoState.SHOOT_2_POS;
                }
                break;

            case SHOOT_2_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_2;
                }
                break;

            case SHOOT_2:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.intakepos3);
                    currentState = AutoState.INTAKE_3_POS;
                }
                break;

            case INTAKE_3_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake3, true);
                    currentState = AutoState.INTAKE_3;
                }
                break;

            case INTAKE_3:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootpos3);
                    currentState = AutoState.SHOOT_3_POS;
                }
                break;

            case SHOOT_3_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_3;
                }
                break;

            case SHOOT_3:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.Leave);
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
                stopIntake();
                if (!follower.isBusy()) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                stopIntake();
                // Main loop will set shooter powers to 0
                break;
        }
    }

    private void runIntakeMode() {
        if (!jammed) {
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(INTAKE_TRANSFER_POWER);
        } else {
            stopIntake();
        }
    }

    private void runShootFeeder() {
        // Clear jam state manually to allow feeding balls into shooter
        jammed = false;
        objectDetected = false;
        intakeMotor.setPower(SHOOT_INTAKE_POWER);
        transferMotor.setPower(SHOOT_TRANSFER_POWER);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    private void updateJamDetection() {
        double d = intakeSensor.getDistance(DistanceUnit.CM);
        if (d < JAM_DISTANCE_CM) {
            if (!objectDetected) {
                objectDetected = true;
                jamTimer.reset();
            } else if (jamTimer.seconds() > JAM_TIME) {
                jammed = true;
            }
        } else {
            objectDetected = false;
            jammed = false;
        }
    }

    @Override
    public void stop() {
        shooterTop.setPower(0);
        shooterBottom.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);

        autoEndPose = follower.getPose();
    }

    public static class Paths {
        public PathChain StartToShoot, Intakepos1, intake1, Shootpos, intakepos2, intake2, shootpos2, intakepos3, intake3, shootpos3, Leave;

        public Paths(Follower follower) {
            Pose blueStart = new Pose(56, 8);
            Pose blueShoot = new Pose(58.579, 13.828);
            Pose blueIntake1Pre = new Pose(54.662, 27.166);
            Pose blueIntake1PreAdjust = new Pose(54.662, 27.166);
            Pose blueIntake1 = new Pose(22, 27.441);
            Pose blueIntake1Adjust = new Pose(22, 27.441);
            Pose blueIntake2Pre = new Pose(58.386, 50.931);
            Pose blueIntake2 = new Pose(16.5, 50.952);
            Pose blueIntake2Adjust = new Pose(28, 56.952);
            Pose blueShoot2 = new Pose(58.607, 13.290);
            Pose blueIntake3Pre = new Pose(58.262, 78.248);
            Pose blueIntake3 = new Pose(26, 78.145);
            Pose blueIntake3Adjust = new Pose(26, 80.145);
            Pose blueShoot3 = new Pose(58.841, 13.221);
            Pose blueLeave = new Pose(50.393, 22.331);

            StartToShoot = follower.pathBuilder().addPath(
                    new BezierLine(blueStart.mirror(), blueShoot.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66.5)).build();

            Intakepos1 = follower.pathBuilder().addPath(
                    new BezierLine(blueShoot.mirror(), blueIntake1Pre.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180)).build();

            intake1 = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake1PreAdjust.mirror(), blueIntake1.mirror())
            ).setTangentHeadingInterpolation().setReversed().build();

            Shootpos = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake1Adjust.mirror(), blueShoot.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(66.5)).build();

            intakepos2 = follower.pathBuilder().addPath(
                    new BezierLine(blueShoot.mirror(), blueIntake2Pre.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180)).build();

            intake2 = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake2Pre.mirror(), blueIntake2.mirror())
            ).setTangentHeadingInterpolation().setReversed().build();

            shootpos2 = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake2Adjust.mirror(), blueShoot2.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(66.5)).build();

            intakepos3 = follower.pathBuilder().addPath(
                    new BezierLine(blueShoot2.mirror(), blueIntake3Pre.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180)).build();

            intake3 = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake3Pre.mirror(), blueIntake3.mirror())
            ).setTangentHeadingInterpolation().setReversed().build();

            shootpos3 = follower.pathBuilder().addPath(
                    new BezierLine(blueIntake3Adjust.mirror(), blueShoot3.mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(65.5)).build();

            Leave = follower.pathBuilder().addPath(
                    new BezierLine(blueShoot3.mirror(), blueLeave.mirror())
            ).setTangentHeadingInterpolation().build();
        }
    }
}