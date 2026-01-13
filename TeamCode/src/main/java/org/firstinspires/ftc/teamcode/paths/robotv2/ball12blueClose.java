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

// Integration with your SubfilesV2
import org.firstinspires.ftc.teamcode.subfilesV2.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subfilesV2.LightingSubsystem; // LIGHTS ADDED

@Autonomous(name = "12 Ball Blue - Full Conversion", group = "Autonomous")
@Configurable
public class ball12blueClose extends OpMode {

    // Pose handoff for TeleOp
    public static Pose autoEndPose = new Pose(0, 0, 0);

    /* ================= SUBSYSTEMS & TOOLS ================= */
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private ShooterSubsystem shooter;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DistanceSensor intakeSensor;
    private LightingSubsystem lights;

    /* ================= TIMERS & VARS ================= */
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();
    private int pathState;

    private boolean objectDetected = false;
    private boolean jammed = false;

    // Constants
    private static final double JAM_DISTANCE_CM = 10.0;
    private static final double JAM_TIME = 0.2;
    private static final double SHOOT_DURATION = 2.0; // 2 seconds per volley

    private enum AutoState {
        PRELOAD_SHOOT,
        INTAKE_1,
        SHOOT_1,
        GATE_INTAKE,
        SHOOT_2,
        INTAKE_2,
        SHOOT_3,
        LEAVE,
        IDLE
    }

    private AutoState currentState = AutoState.PRELOAD_SHOOT;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // 1. Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.779, 109.959, Math.toRadians(90)));

        // 2. Initialize Paths
        paths = new Paths(follower);

        // 3. Initialize Intake Hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 4. Initialize Shooter Subsystem (isRed = false for Blue)
        shooter = new ShooterSubsystem(hardwareMap, false);
        lights = new LightingSubsystem(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized - Blue 12 Ball");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.enable();
        shooter.setCloseMode(); // Use close mode presets + your ML adjustments
        shootTimer.reset();
        lights.setShooterLight(LightingSubsystem.LightMode.BLUE); // Set Team Color
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(); // Keep PID active
        updateJamDetection();
        autonomousPathUpdate();
        lights.update();
        updateLightsLogic();
        // Telemetry
        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("Shooter RPM", shooter.getCurrentRPM());
        panelsTelemetry.debug("Jammed", jammed);
        panelsTelemetry.debug("Pose", follower.getPose().toString());
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
                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1);
                    currentState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                stopIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    shootTimer.reset();
                    currentState = AutoState.GATE_INTAKE;
                }
                break;

            case GATE_INTAKE:
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    // Drive to gate - Full power intake (ignoring jam sensor here)
                    intakeMotor.setPower(-1.0);
                    transferMotor.setPower(-0.15);
                    if (!follower.isBusy()) {
                        follower.followPath(paths.gateintake);
                        currentState = AutoState.SHOOT_2;
                    }
                }
                break;

            case SHOOT_2:
                if (!follower.isBusy()) {
                    stopIntake();
                    follower.followPath(paths.shoot2);
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    runIntakeMode();
                    if (!follower.isBusy()) {
                        follower.followPath(paths.intake);
                        currentState = AutoState.SHOOT_3;
                    }
                }
                break;

            case SHOOT_3:
                stopIntake();
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3);
                    shootTimer.reset();
                    currentState = AutoState.LEAVE;
                }
                break;

            case LEAVE:
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    shooter.disable();
                    if (!follower.isBusy()) {
                        follower.followPath(paths.leave);
                        currentState = AutoState.IDLE;
                    }
                }
                break;

            case IDLE:
                stopIntake();
                shooter.disable();
                break;
        }
        pathState = currentState.ordinal();
    }

    /* ================= HELPERS ================= */

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
        transferMotor.setPower(0.9);
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
        autoEndPose = follower.getPose(); // Pass pose to TeleOp
        shooter.stop(); // Save any adjustments
    }

    /* ================= PATHS ================= */
    public static class Paths {
        public PathChain shoot0, intake1, shoot1, gateintake, shoot2, intake, shoot3, leave;

        public Paths(Follower follower) {
            shoot0 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15.779, 109.959), new Pose(57.655, 78.207))
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(129)).setReversed().build();

            intake1 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(57.655, 78.207), new Pose(56.858, 59.569), new Pose(22.441, 59.469))
            ).setTangentHeadingInterpolation().setReversed().build();

            shoot1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(22.441, 59.469), new Pose(58.497, 77.745))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(129)).build();

            gateintake = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.497, 77.745), new Pose(11.103, 59.469))
            ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(327)).build();

            shoot2 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(11.103, 59.469), new Pose(73.931, 57.838), new Pose(58.262, 77.876))
            ).setTangentHeadingInterpolation().build();

            intake = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(58.262, 77.876), new Pose(38.024, 82.462), new Pose(20.379, 83.738))
            ).setTangentHeadingInterpolation().build();

            shoot3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(20.379, 83.738), new Pose(40.410, 99.810), new Pose(58.124, 77.869))
            ).setLinearHeadingInterpolation(Math.toRadians(176), Math.toRadians(129)).setReversed().build();

            leave = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(58.124, 77.869), new Pose(50.814, 72.738))
            ).setConstantHeadingInterpolation(Math.toRadians(129)).build();
        }
    }
}