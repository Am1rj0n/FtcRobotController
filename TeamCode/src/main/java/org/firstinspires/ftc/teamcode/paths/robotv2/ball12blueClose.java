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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.subfilesV2.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subfilesV2.LightingSubsystem;

@Autonomous(name = "12 Ball Blue - CLOSE", group = "Autonomous")
@Configurable
public class ball12blueClose extends OpMode {

    public static Pose autoEndPose = new Pose(0, 0, 0);

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private ShooterSubsystem shooter;
    private DcMotor intakeMotor, transferMotor;
    private DistanceSensor intakeSensor;
    private LightingSubsystem lights;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();

    private boolean objectDetected = false;
    private boolean jammed = false;

    // Motor Power Constants
    private static final double INTAKE_POWER = -1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.15;
    private static final double SPIT_INTAKE_POWER = 0.9;
    private static final double SPIT_TRANSFER_POWER = -0.85;
    private static final double SHOOT_INTAKE_POWER = -0.55;
    private static final double SHOOT_TRANSFER_POWER = 0.5;

    // Tuning Constants
    private static final double JAM_DISTANCE_CM = 12.0;
    private static final double JAM_TIME = 0.46;
    private static final double SHOOT_DURATION = 2.0;

    private enum AutoState {
        PRELOAD_POS, PRELOAD_SHOOT,
        INTAKE_1_POS, INTAKE_1, SHOOT_1_POS, SHOOT_1,
        GATE_SHOOT_POS, GATE_SHOOT, GATE_INTAKE,
        SHOOT_2_POS, SHOOT_2,
        INTAKE_2_POS, INTAKE_2, SHOOT_3_POS, SHOOT_3,
        LEAVE, IDLE
    }
    private AutoState currentState = AutoState.PRELOAD_POS;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.779, 109.959, Math.toRadians(90)));
        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter = new ShooterSubsystem(hardwareMap, false);
        lights = new LightingSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        shooter.enable();
        shooter.setCloseMode();
        follower.followPath(paths.shoot0);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        updateJamDetection();
        autonomousPathUpdate();
        lights.update();
        updateLightsLogic();

        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("Progress", follower.getCurrentTValue());
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        double t = follower.getCurrentTValue();

        switch (currentState) {
            case PRELOAD_POS:
                stopIntake();
                if (!follower.isBusy()) {
                    currentState = AutoState.PRELOAD_SHOOT;
                    shootTimer.reset();
                }
                break;

            case PRELOAD_SHOOT:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.intake1Pos);
                    currentState = AutoState.INTAKE_1_POS;
                }
                break;

            case INTAKE_1_POS:
                runIntakeMode();
                if (t > 0.5) shooter.setCloseMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1, true);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    currentState = AutoState.SHOOT_1_POS;
                }
                break;

            case SHOOT_1_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    currentState = AutoState.SHOOT_1;
                    shootTimer.reset();
                }
                break;

            case SHOOT_1:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.gateShootPos);
                    currentState = AutoState.GATE_SHOOT_POS;
                }
                break;

            case GATE_SHOOT_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    currentState = AutoState.GATE_SHOOT;
                    shootTimer.reset();
                }
                break;

            case GATE_SHOOT:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.gateintake);
                    currentState = AutoState.GATE_INTAKE;
                }
                break;

            case GATE_INTAKE:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2);
                    currentState = AutoState.SHOOT_2_POS;
                }
                break;

            case SHOOT_2_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    currentState = AutoState.SHOOT_2;
                    shootTimer.reset();
                }
                break;

            case SHOOT_2:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.intake2Pos);
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
                    follower.followPath(paths.shoot3);
                    currentState = AutoState.SHOOT_3_POS;
                }
                break;

            case SHOOT_3_POS:
                runIntakeMode();
                if (!follower.isBusy()) {
                    currentState = AutoState.SHOOT_3;
                    shootTimer.reset();
                }
                break;

            case SHOOT_3:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.leave);
                    currentState = AutoState.LEAVE;
                }
                break;

            case LEAVE:
                stopIntake();
                if (!follower.isBusy()) {
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
        if (!jammed) {
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(INTAKE_TRANSFER_POWER);
        } else {
            stopIntake();
        }
    }

    private void runShootMode() {
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

    private void updateLightsLogic() {
        if (jammed) {
            lights.setJamLight(LightingSubsystem.LightMode.RED_BLINK);
        } else {
            lights.setJamLight(LightingSubsystem.LightMode.GREEN);
        }

        if (shooter.isEnabled() && shooter.isAtTarget()) {
            lights.setShooterLight(LightingSubsystem.LightMode.PINK_BLINK);
        } else {
            lights.setShooterLight(LightingSubsystem.LightMode.BLUE);
        }
    }

    @Override
    public void stop() {
        autoEndPose = follower.getPose();
        shooter.stop();
    }

    public static class Paths {
        public PathChain shoot0, intake1Pos, intake1, shoot1, gateShootPos, gateintake, shoot2, intake2Pos, intake2, shoot3, leave;

        public Paths(Follower follower) {
            shoot0 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.779, 109.959), new Pose(57.655, 78.207)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(129))
                    .build();

            intake1Pos = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(57.655, 78.207), new Pose(56.858, 59.569), new Pose(22.441, 59.469)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22.441, 59.469), new Pose(22.441, 59.469)))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(22.441, 59.469), new Pose(58.497, 77.745)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(129))
                    .build();

            gateShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.497, 77.745), new Pose(58.497, 77.745)))
                    .setConstantHeadingInterpolation(Math.toRadians(129))
                    .build();

            gateintake = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.497, 77.745), new Pose(11.103, 59.469)))
                    .setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(327))
                    .setReversed()
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(11.103, 59.469), new Pose(73.931, 57.838), new Pose(58.262, 77.876)))
                    .setTangentHeadingInterpolation()
                    .build();

            intake2Pos = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58.262, 77.876), new Pose(38.024, 82.462), new Pose(20.379, 83.738)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(20.379, 83.738), new Pose(20.379, 83.738)))
                    .setConstantHeadingInterpolation(Math.toRadians(176))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(20.379, 83.738), new Pose(40.410, 99.810), new Pose(58.124, 77.869)))
                    .setLinearHeadingInterpolation(Math.toRadians(176), Math.toRadians(129))
                    .build();

            leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.124, 77.869), new Pose(50.814, 72.738)))
                    .setConstantHeadingInterpolation(Math.toRadians(129))
                    .build();
        }
    }
}