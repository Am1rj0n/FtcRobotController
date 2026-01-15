package org.firstinspires.ftc.teamcode.AutoCAD;

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
import org.firstinspires.ftc.teamcode.subsystems.LimelightHelper;

@Autonomous(name = "Scan Auto Red - V3", group = "Autonomous")
@Configurable
public class ScanRed extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private LimelightHelper limelight;

    // Intake/Transfer Hardware
    private DcMotor intakeMotor, transferMotor;
    private DistanceSensor intakeSensor;

    // Shooter Hardware (Controlled directly in Auto)
    private DcMotor shooterTop, shooterBottom;
    private LightingSubsystem lights;

    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime scanTimer = new ElapsedTime();
    private ElapsedTime jamTimer = new ElapsedTime();

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
    private static final double SHOOT_DURATION = 2.5;
    private static final double JAM_DISTANCE_CM = 14.0;
    private static final double JAM_TIME = 0.7;
    private static final double SCAN_TIMEOUT = 2.0;

    private boolean objectDetected = false;
    private boolean jammed = false;
    private int detectedTagId = -1;

    // Blue side waypoints (will be mirrored for red)
    private static final Pose blueStartPos = new Pose(56.000, 8.000, Math.toRadians(90));
    private static final Pose blueShootPos = new Pose(58.579, 13.828, Math.toRadians(113.5));
    private static final Pose blueScanPos = new Pose(57.821, 34.841, Math.toRadians(90));

    // Dynamic paths (built after scan)
    private PathChain IntakePathChain, ReturnShootPath;

    private enum AutoState {
        SPIN_UP, START_TO_SCAN, SCANNING,
        GO_TO_SHOOT, SHOOT_PRELOAD,
        GO_TO_INTAKE, INTAKE_BALLS,
        RETURN_TO_SHOOT, FINAL_SHOOT,
        IDLE
    }
    private AutoState currentState = AutoState.SPIN_UP;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blueStartPos.mirror());

        limelight = new LimelightHelper(hardwareMap);

        // Intake/Transfer Init
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        // Shooter Motor Init
        shooterTop = hardwareMap.get(DcMotor.class, "outtakeMotor2");
        shooterBottom = hardwareMap.get(DcMotor.class, "outtakeMotor1");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);

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
            shooterTop.setPower(TOP_MOTOR_POWER);
            shooterBottom.setPower(BOTTOM_MOTOR_POWER);
        } else {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }

        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("Detected Tag", detectedTagId);
        panelsTelemetry.debug("Shooter Top", shooterTop.getPower());
        panelsTelemetry.debug("Shooter Bottom", shooterBottom.getPower());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case SPIN_UP:
                stopIntake();
                if (shootTimer.seconds() > SPIN_UP_DURATION) {
                    // Go directly to scan position
                    PathChain toScan = follower.pathBuilder()
                            .addPath(new BezierLine(blueStartPos.mirror(), blueScanPos.mirror()))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                            .build();
                    follower.followPath(toScan);
                    currentState = AutoState.START_TO_SCAN;
                }
                break;

            case START_TO_SCAN:
                stopIntake();
                if (!follower.isBusy()) {
                    scanTimer.reset();
                    currentState = AutoState.SCANNING;
                }
                break;

            case SCANNING:
                stopIntake();
                detectedTagId = limelight.detectAprilTag();
                if ((detectedTagId >= 21 && detectedTagId <= 23) || scanTimer.seconds() > SCAN_TIMEOUT) {
                    if (detectedTagId == -1 || detectedTagId < 21 || detectedTagId > 23) {
                        detectedTagId = 22; // Default to middle
                    }
                    buildDynamicPaths(detectedTagId);

                    // Go to shoot position
                    PathChain toShoot = follower.pathBuilder()
                            .addPath(new BezierLine(blueScanPos.mirror(), blueShootPos.mirror()))
                            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66.5))
                            .build();
                    follower.followPath(toShoot);
                    currentState = AutoState.GO_TO_SHOOT;
                }
                break;

            case GO_TO_SHOOT:
                stopIntake();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    follower.followPath(IntakePathChain, true);
                    currentState = AutoState.GO_TO_INTAKE;
                }
                break;

            case GO_TO_INTAKE:
                runIntakeMode();
                if (!follower.isBusy()) {
                    currentState = AutoState.INTAKE_BALLS;
                }
                break;

            case INTAKE_BALLS:
                runIntakeMode();
                // Continue intaking for a brief moment at the end position
                if (shootTimer.seconds() > SHOOT_DURATION + 0.5) {
                    follower.followPath(ReturnShootPath);
                    currentState = AutoState.RETURN_TO_SHOOT;
                }
                break;

            case RETURN_TO_SHOOT:
                runIntakeMode();
                if (!follower.isBusy()) {
                    shootTimer.reset();
                    currentState = AutoState.FINAL_SHOOT;
                }
                break;

            case FINAL_SHOOT:
                runShootFeeder();
                if (shootTimer.seconds() > SHOOT_DURATION) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                stopIntake();
                break;
        }
    }

    private void buildDynamicPaths(int tagId) {
        Pose blueIntakePrePos, blueIntakePreAdjust, blueIntakeAdjust, blueIntakeFinal, blueShootReturn;

        if (tagId == 21) {
            // ID 21 paths (mirrored)
            blueIntakePrePos = new Pose(58.579, 13.828);
            blueIntakePreAdjust = new Pose(54.662, 30.166);
            blueIntakeAdjust = new Pose(54.662, 30.166);
            blueIntakeFinal = new Pose(22, 30.441);
            blueShootReturn = new Pose(58.579, 13.828);

            IntakePathChain = follower.pathBuilder()
                    .addPath(new BezierLine(blueIntakePrePos.mirror(), blueIntakePreAdjust.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180))
                    .addPath(new BezierLine(blueIntakeAdjust.mirror(), blueIntakeFinal.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ReturnShootPath = follower.pathBuilder()
                    .addPath(new BezierLine(blueIntakeFinal.mirror(), blueShootReturn.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(66.5))
                    .build();

        } else if (tagId == 22) {
            // ID 22 paths (mirrored)
            blueIntakePrePos = new Pose(58.579, 13.828);
            blueIntakePreAdjust = new Pose(58.386, 50.931);
            blueIntakeFinal = new Pose(16.5, 50.952);
            blueShootReturn = new Pose(58.607, 13.290);

            IntakePathChain = follower.pathBuilder()
                    .addPath(new BezierLine(blueIntakePrePos.mirror(), blueIntakePreAdjust.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180))
                    .addPath(new BezierLine(blueIntakePreAdjust.mirror(), blueIntakeFinal.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pose blueAdjustedReturn = new Pose(28, 56.952);
            ReturnShootPath = follower.pathBuilder()
                    .addPath(new BezierLine(blueAdjustedReturn.mirror(), blueShootReturn.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(66.5))
                    .build();

        } else { // tagId == 23
            // ID 23 paths (mirrored)
            blueIntakePrePos = new Pose(58.607, 13.290);
            blueIntakePreAdjust = new Pose(58.262, 78.248);
            blueIntakeFinal = new Pose(26, 78.145);
            blueShootReturn = new Pose(58.841, 13.221);

            IntakePathChain = follower.pathBuilder()
                    .addPath(new BezierLine(blueIntakePrePos.mirror(), blueIntakePreAdjust.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(66.5), Math.toRadians(180))
                    .addPath(new BezierLine(blueIntakePreAdjust.mirror(), blueIntakeFinal.mirror()))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pose blueAdjustedReturn = new Pose(26, 80.145);
            ReturnShootPath = follower.pathBuilder()
                    .addPath(new BezierLine(blueAdjustedReturn.mirror(), blueShootReturn.mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(65.5))
                    .build();
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
        limelight.close();
    }
}