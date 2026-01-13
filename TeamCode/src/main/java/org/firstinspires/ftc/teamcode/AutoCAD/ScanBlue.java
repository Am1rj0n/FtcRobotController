package org.firstinspires.ftc.teamcode.AutoCAD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightHelper;

@Autonomous(name = "Obelisk Scan Auto - BLUE", group = "Auto")
public class ScanBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private LimelightHelper limelight;

    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx outtakeMotor1, outtakeMotor2;
    private IMU imu;

    private int pathState = 0;
    private double speed = 1.0;

    // Timeout for path completion
    private static final double PATH_TIMEOUT_MS = 10000;
    private static final double SCAN_TIMEOUT_MS = 2000;

    // System Modes
    private enum SystemMode { OFF, INTAKE_BALLS, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    // Detected AprilTag ID
    private int detectedTagId = -1;

    // Shot power settings
    private static final double OUTTAKE_MOTOR1_POWER = 0.75;
    private static final double OUTTAKE_MOTOR2_POWER = 0.80;
    private final int SHOOTER_SPINUP_MS = 3000;
    private final int SHOOT_DURATION_MS = 2500;

    private boolean shootingActive = false;
    private ElapsedTime shootTimer = new ElapsedTime();

    // Path flags
    private boolean scanStarted = false, scanComplete = false;
    private boolean intakeStarted = false, intakeComplete = false;
    private boolean shootPosStarted = false, shootPosComplete = false;
    private boolean shootingStarted = false, shootingComplete = false;

    // Waypoints (BLUE side)
    private static final Pose startPos = new Pose(56.000, 8.000, Math.toRadians(90));
    private static final Pose scanPos = new Pose(57.821, 34.841, Math.toRadians(90));
    private static final Pose shootPos = new Pose(56.890, 12.214, Math.toRadians(110));

    // Intake positions for each tag (BLUE side)
    private static final Pose intakeID21 = new Pose(11.014, 36.110, Math.toRadians(0));
    private static final Pose intakeID22 = new Pose(18.131, 58.455, Math.toRadians(0));
    private static final Pose intakeID23 = new Pose(16.972, 83.448, Math.toRadians(0));

    private PathChain ScanPosition, IntakePosition, ShootPosition;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Limelight
        limelight = new LimelightHelper(hardwareMap);

        // Set motor directions
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPos);

        // Build initial scan path
        buildScanPath();

        telemetry.addData("Status", "Initialized - Obelisk Scan Auto BLUE");
        telemetry.addLine("Limelight ready for AprilTag detection");
        telemetry.update();
    }

    private void buildScanPath() {
        ScanPosition = follower.pathBuilder()
                .addPath(new BezierLine(startPos, scanPos))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    private void buildIntakePath(int tagId) {
        Pose intakeTarget;

        switch (tagId) {
            case 21:
                intakeTarget = intakeID21;
                break;
            case 22:
                intakeTarget = intakeID22;
                break;
            case 23:
                intakeTarget = intakeID23;
                break;
            default:
                intakeTarget = intakeID22; // Default to ID22 if unknown
                break;
        }

        IntakePosition = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scanPos,
                        new Pose(63.624, 63.703, Math.toRadians(0)),
                        new Pose(45.272, 58.421, Math.toRadians(0)),
                        intakeTarget
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    private void buildShootPath(Pose fromPose) {
        ShootPosition = follower.pathBuilder()
                .addPath(new BezierLine(fromPose, shootPos))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                .build();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setMaxPower(speed);

        applySystemMode();
        autonomousPathUpdate();

        telemetry.addData("State", getStateDescription());
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Detected Tag", detectedTagId == -1 ? "None" : "ID " + detectedTagId);
        telemetry.addData("Path Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Speed", speed);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        boolean timedOut = pathTimer.getElapsedTime() > PATH_TIMEOUT_MS;

        switch (pathState) {
            case 0: // Move to scan position
                if (!scanStarted) {
                    speed = 1.0;
                    follower.followPath(ScanPosition, true);
                    scanStarted = true;
                }
                if ((scanStarted && !follower.isBusy() && poseCloseTo(follower.getPose(), scanPos, 3.0)) || timedOut) {
                    setPathState(1);
                }
                break;

            case 1: // Scan for AprilTag
                if (!scanComplete) {
                    // Continuously poll for tags
                    detectedTagId = limelight.detectAprilTag();

                    if (detectedTagId >= 21 && detectedTagId <= 23) {
                        // Valid tag detected
                        buildIntakePath(detectedTagId);
                        scanComplete = true;
                        setPathState(2);
                    } else if (pathTimer.getElapsedTime() > SCAN_TIMEOUT_MS) {
                        // Timeout - default to ID22
                        detectedTagId = 22;
                        buildIntakePath(detectedTagId);
                        scanComplete = true;
                        setPathState(2);
                    }

                    // Add telemetry to debug
                    telemetry.addData("Scanning", "Looking for tags...");
                    telemetry.addData("Has Detection", limelight.hasDetection());
                }
                break;

            case 2: // Move to intake position (with intake running)
                if (!intakeStarted) {
                    speed = 0.7;
                    currentMode = SystemMode.INTAKE_BALLS;
                    follower.followPath(IntakePosition, true);
                    intakeStarted = true;
                }

                Pose intakeTarget = getIntakeTarget();
                if ((intakeStarted && !follower.isBusy() && poseCloseTo(follower.getPose(), intakeTarget, 3.0)) || timedOut) {
                    currentMode = SystemMode.OFF;
                    intakeComplete = true;

                    // Build shoot path from current position
                    buildShootPath(follower.getPose());
                    setPathState(3);
                }
                break;

            case 3: // Move to shoot position
                if (!shootPosStarted) {
                    speed = 1.0;
                    follower.followPath(ShootPosition, true);
                    shootPosStarted = true;
                }
                if ((shootPosStarted && !follower.isBusy() && poseCloseTo(follower.getPose(), shootPos, 4.0)) || timedOut) {
                    shootPosComplete = true;
                    setPathState(4);
                }
                break;

            case 4: // Shoot (3 sec spinup, then shoot)
                if (!shootingActive) {
                    startShooting();
                }
                if (continueShooting()) {
                    setPathState(-1); // Done
                }
                break;
        }
    }

    private Pose getIntakeTarget() {
        switch (detectedTagId) {
            case 21: return intakeID21;
            case 22: return intakeID22;
            case 23: return intakeID23;
            default: return intakeID22;
        }
    }

    private void startShooting() {
        shootingActive = true;
        outtakeMotor1.setPower(OUTTAKE_MOTOR1_POWER);
        outtakeMotor2.setPower(OUTTAKE_MOTOR2_POWER);
        shootTimer.reset();
    }

    private boolean continueShooting() {
        if (!shootingActive) return false;

        // Spin up for 3 seconds
        if (shootTimer.milliseconds() < SHOOTER_SPINUP_MS) {
            currentMode = SystemMode.OFF;
            return false;
        }

        // After spinup, start feeding balls
        currentMode = SystemMode.SHOOT;

        // Continue shooting for the remaining duration
        if (shootTimer.milliseconds() < SHOOTER_SPINUP_MS + SHOOT_DURATION_MS) {
            return false;
        }

        // Done shooting
        shootingActive = false;
        currentMode = SystemMode.OFF;
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        return true;
    }

    private void applySystemMode() {
        switch (currentMode) {
            case INTAKE_BALLS:
                intakeMotor.setPower(-1.0);
                transferMotor.setPower(-0.15);
                break;
            case SHOOT:
                intakeMotor.setPower(-0.55);
                transferMotor.setPower(0.9);
                break;
            case OFF:
            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                if (!shootingActive) {
                    outtakeMotor1.setPower(0);
                    outtakeMotor2.setPower(0);
                }
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    private String getStateDescription() {
        switch (pathState) {
            case 0: return "MOVE TO SCAN";
            case 1: return "SCANNING OBELISK";
            case 2: return "INTAKE (MOVING)";
            case 3: return "MOVE TO SHOOT";
            case 4: return "SHOOTING";
            default: return "COMPLETE";
        }
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        limelight.close();
    }

    private boolean poseCloseTo(Pose current, Pose target, double tolInches) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy) <= tolInches;
    }
}