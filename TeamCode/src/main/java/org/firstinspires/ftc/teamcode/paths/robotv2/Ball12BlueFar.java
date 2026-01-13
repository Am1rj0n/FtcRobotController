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
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subfilesV2.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subfilesV2.LightingSubsystem;
@Autonomous(name = "12 Ball Blue New", group = "Autonomous")
@Configurable
public class Ball12BlueFar extends OpMode {

    // Pose for TeleOp handoff
    public static Pose autoEndPose = new Pose(0, 0, 0);

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Hardware
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DistanceSensor intakeSensor;
    private ShooterSubsystem shooter;

    private LightingSubsystem lights;

    // Timers
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private static final double SHOOT_DURATION = 2.5; // 2500ms for shooting

    // Jam Detection
    private boolean objectDetected = false;
    private boolean jammed = false;
    private static final double JAM_DISTANCE_CM = 10.0;
    private static final double JAM_TIME = 0.2;

    private enum AutoState {
        SHOOT_PRELOAD,  // Initial Shoot (ShootPos0)
        DRIVE_TO_INTAKE_1,
        INTAKE_SAMPLE_1,
        DRIVE_TO_SHOOT_1,
        SHOOT_SAMPLE_1,
        DRIVE_TO_INTAKE_2,
        INTAKE_SAMPLE_2,
        DRIVE_TO_SHOOT_2,
        SHOOT_SAMPLE_2,
        DRIVE_TO_INTAKE_3,
        INTAKE_SAMPLE_3,
        DRIVE_TO_SHOOT_3,
        SHOOT_SAMPLE_3,
        PARK,
        IDLE
    }

    private AutoState currentState = AutoState.SHOOT_PRELOAD;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Start position from your first Path line (72, 8) or (56, 8) depending on preference
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        // Use your ShooterSubsystem from subfilesV2
        shooter = new ShooterSubsystem(hardwareMap, false);
        lights = new LightingSubsystem(hardwareMap); // LIGHTS ADDED

        panelsTelemetry.debug("Status", "Auto Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shootTimer.reset();
        shooter.enable();
        shooter.setFarMode();
        lights.setShooterLight(LightingSubsystem.LightMode.BLUE);
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
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    private void updateLightsLogic() {
        // Handle Jam Light
        if (jammed) {
            lights.setJamLight(LightingSubsystem.LightMode.RED_BLINK);
        } else {
            lights.setJamLight(LightingSubsystem.LightMode.GREEN);
        }

        // Handle Shooter Light (Blink when at target RPM)
        if (shooter.isEnabled() && shooter.isAtTarget()) {
            lights.setShooterLight(LightingSubsystem.LightMode.PINK_BLINK); // "Ready to Fire"
        } else {
            lights.setShooterLight(LightingSubsystem.LightMode.BLUE); // "Charging/Normal"
        }
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case SHOOT_PRELOAD:
                // Stay put and shoot preload
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    stopIntake();
                    follower.followPath(paths.intake1Pos);
                    currentState = AutoState.DRIVE_TO_INTAKE_1;
                }
                break;

            case DRIVE_TO_INTAKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intake1);
                    currentState = AutoState.INTAKE_SAMPLE_1;
                }
                break;

            case INTAKE_SAMPLE_1:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPos1);
                    currentState = AutoState.DRIVE_TO_SHOOT_1;
                }
                break;

            case DRIVE_TO_SHOOT_1:
                stopIntake();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_SAMPLE_1;
                }
                break;

            case SHOOT_SAMPLE_1:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    stopIntake();
                    follower.followPath(paths.Intake2);
                    currentState = AutoState.DRIVE_TO_INTAKE_2;
                }
                break;

            case DRIVE_TO_INTAKE_2:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootPos2);
                    currentState = AutoState.DRIVE_TO_SHOOT_2;
                }
                break;

            case DRIVE_TO_SHOOT_2:
                stopIntake();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_SAMPLE_2;
                }
                break;

            case SHOOT_SAMPLE_2:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    stopIntake();
                    follower.followPath(paths.Intake3);
                    currentState = AutoState.DRIVE_TO_INTAKE_3;
                }
                break;

            case DRIVE_TO_INTAKE_3:
                runIntakeMode();
                if (!follower.isBusy()) {
                    follower.followPath(paths.SHootPos3);
                    currentState = AutoState.DRIVE_TO_SHOOT_3;
                }
                break;

            case DRIVE_TO_SHOOT_3:
                stopIntake();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_SAMPLE_3;
                }
                break;

            case SHOOT_SAMPLE_3:
                runShootMode();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    stopIntake();
                    follower.followPath(paths.Leave);
                    currentState = AutoState.PARK;
                }
                break;

            case PARK:
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
            intakeMotor.setPower(-1.0);
            transferMotor.setPower(-0.15);
        } else {
            stopIntake();
        }
    }

    private void runShootMode() {
        intakeMotor.setPower(-0.55);
        transferMotor.setPower(0.9); // Your literal shoot power
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
        autoEndPose = follower.getPose();
    }

    public static class Paths {
        public PathChain intake1Pos, Intake1, ShootPos1, Intake2, ShootPos2, Intake3, SHootPos3, Leave;

        public Paths(Follower follower) {
            intake1Pos = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(56, 8), new Pose(55.669, 36.497)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            Intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.669, 36.497), new Pose(13.6, 36.103)))
                    .setTangentHeadingInterpolation().setReversed()
                    .build();

            ShootPos1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(13.6, 36.103), new Pose(58.641, 13.545)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(111))
                    .build();

            Intake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58.641, 13.545), new Pose(64.862, 60.507), new Pose(19.414, 59.855)))
                    .setTangentHeadingInterpolation().setReversed()
                    .build();

            ShootPos2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.414, 59.855), new Pose(58.559, 13.621)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(111))
                    .build();

            Intake3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(58.559, 13.621), new Pose(75.821, 84.021), new Pose(19.648, 83.372)))
                    .setTangentHeadingInterpolation().setReversed()
                    .build();

            SHootPos3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(19.648, 83.372), new Pose(58.986, 13.779)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(111))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(58.986, 13.779), new Pose(51.345, 21.779)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}