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

@Autonomous(name = "12 Ball Red - Close", group = "Autonomous")
@Configurable
public class ball12redClose extends OpMode {

    // Sharing the same static pose variable for TeleOp handoff
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

    // Tuning Constants
    private static final double JAM_DISTANCE_CM = 12.0;
    private static final double JAM_TIME = 0.46;
    private static final double SHOOT_DURATION = 2.0;
    private static final double TRANSFER_SHOOT = 0.70;
    private static final double TRANSFER_IDLE = 0.10;

    private enum AutoState { PRELOAD_SHOOT, INTAKE_1, SHOOT_1, GATE_INTAKE, SHOOT_2, INTAKE_2, SHOOT_3, LEAVE, IDLE }
    private AutoState currentState = AutoState.PRELOAD_SHOOT;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Red Starting Pose (Mirrored from Blue)
        Pose startPose = new Pose(15.779, 109.959, Math.toRadians(90)).mirror();
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // isRed = true for this file
        shooter = new ShooterSubsystem(hardwareMap, true);
        lights = new LightingSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        shooter.enable();
        shooter.setCloseMode();
        shootTimer.reset();
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
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        double t = follower.getCurrentTValue();

        switch (currentState) {
            case PRELOAD_SHOOT:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    stopIntake();
                    follower.followPath(paths.shoot0);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                runIntakeMode();
                if (t > 0.5) shooter.setCloseMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1, true);
                    currentState = AutoState.SHOOT_1;
                    shootTimer.reset();
                }
                break;

            case SHOOT_1:
                if (t > 0.90 || !follower.isBusy()) runShootMode();
                else stopIntake();

                if (!follower.isBusy() && shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.shoot1);
                    currentState = AutoState.GATE_INTAKE;
                    shootTimer.reset();
                }
                break;

            case GATE_INTAKE:
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    runIntakeMode();
                    if (!follower.isBusy()) {
                        follower.followPath(paths.gateintake);
                        currentState = AutoState.SHOOT_2;
                        shootTimer.reset();
                    }
                }
                break;

            case SHOOT_2:
                if (t > 0.90 || !follower.isBusy()) runShootMode();
                else stopIntake();

                if (!follower.isBusy() && shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.shoot2);
                    currentState = AutoState.INTAKE_2;
                    shootTimer.reset();
                }
                break;

            case INTAKE_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake, true);
                    currentState = AutoState.SHOOT_3;
                    shootTimer.reset();
                }
                break;

            case SHOOT_3:
                if (t > 0.90 || !follower.isBusy()) runShootMode();
                else stopIntake();

                if (!follower.isBusy() && shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(paths.shoot3);
                    currentState = AutoState.LEAVE;
                    shootTimer.reset();
                }
                break;

            case LEAVE:
                stopIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.leave);
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
            intakeMotor.setPower(-1.0);
            transferMotor.setPower(-0.15);
        } else stopIntake();
    }

    private void runShootMode() {
        intakeMotor.setPower(-0.55);
        transferMotor.setPower(TRANSFER_SHOOT);
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        transferMotor.setPower(TRANSFER_IDLE);
    }

    private void updateJamDetection() {
        double d = intakeSensor.getDistance(DistanceUnit.CM);
        if (d < JAM_DISTANCE_CM) {
            if (!objectDetected) { objectDetected = true; jamTimer.reset(); }
            else if (jamTimer.seconds() > JAM_TIME) jammed = true;
        } else { objectDetected = false; jammed = false; }
    }

    private void updateLightsLogic() {
        if (jammed) lights.setJamLight(LightingSubsystem.LightMode.RED_BLINK);
        else lights.setJamLight(LightingSubsystem.LightMode.GREEN);

        if (shooter.isEnabled() && shooter.isAtTarget()) lights.setShooterLight(LightingSubsystem.LightMode.PINK_BLINK);
        else lights.setShooterLight(LightingSubsystem.LightMode.BLUE);
    }

    @Override
    public void stop() {
        // Updated to use this local class static variable for TeleOp handoff
        ball12blueClose.autoEndPose = follower.getPose();
        shooter.stop();
    }

    public static class Paths {
        public PathChain shoot0, intake1, shoot1, gateintake, shoot2, intake, shoot3, leave;
        public Paths(Follower follower) {
            // Using pose.mirror() on all points to flip them across the field centerline
            shoot0 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15.779, 109.959).mirror(), new Pose(57.655, 78.207).mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(129)).build(); // Note: Heading logic usually handles mirror inside setLinearHeading if pose is mirrored

            intake1 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(57.655, 78.207).mirror(), new Pose(56.858, 59.569).mirror(), new Pose(22.441, 59.469).mirror())
            ).setTangentHeadingInterpolation().setReversed().build();

            shoot1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(22.441, 59.469).mirror(), new Pose(58.497, 77.745).mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(129)).build();

            gateintake = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.497, 77.745).mirror(), new Pose(11.103, 59.469).mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(327)).build();

            shoot2 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(11.103, 59.469).mirror(), new Pose(73.931, 57.838).mirror(), new Pose(58.262, 77.876).mirror())
            ).setTangentHeadingInterpolation().build();

            intake = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(58.262, 77.876).mirror(), new Pose(38.024, 82.462).mirror(), new Pose(20.379, 83.738).mirror())
            ).setTangentHeadingInterpolation().setReversed().build();

            shoot3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(20.379, 83.738).mirror(), new Pose(40.410, 99.810).mirror(), new Pose(58.124, 77.869).mirror())
            ).setLinearHeadingInterpolation(Math.toRadians(176), Math.toRadians(129)).build();

            leave = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.124, 77.869).mirror(), new Pose(50.814, 72.738).mirror())
            ).setConstantHeadingInterpolation(Math.toRadians(129)).build();
        }
    }
}