package org.firstinspires.ftc.teamcode.paths.red;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "3 Ball RED LEAVE - Rapid Fire", group = "Auto")
public class AutoLeaveRed extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx outtakeMotor1, outtakeMotor2;
    private IMU imu;

    private ShooterPIDController shooterPID;

    private int pathState = 0;
    private double speed = 1.0;

    // Timeout for path completion
    private static final double PATH_TIMEOUT_MS = 3500;

    // System Modes
    private enum SystemMode { OFF, SPITTER, INTAKE_BALLS, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    // Shot power and timing
    private static final double SHOOT_POWER = 0.80;  // 80% of max RPM
    private static final double MAX_RPM = 5800.0;
    private final int SHOOTER_SPINUP_MS = 2500;
    private final int RAPID_SHOOT_DURATION_MS = 2500;

    private boolean shootingActive = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private double currentTargetRPM = 0;

    // Path flags
    private boolean preloadShot = false;
    private boolean leaveStarted = false;

    // Waypoints - Mirrored for RED
    private static final Pose startPos = new Pose(56.000, 8.000, Math.toRadians(90)).mirror();
    private static final Pose shotPos = new Pose(57.931, 13.903, Math.toRadians(114)).mirror();
    private static final Pose leavePos = new Pose(35.752, 13.903, Math.toRadians(0)).mirror();

    private PathChain ShootPosition0, LeavePosition;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
        imu = hardwareMap.get(IMU.class, "imu");

        shooterPID = new ShooterPIDController();
        shooterPID.setPID(0.008, 0.0003, 0.0001);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPos);
        buildPaths();

        telemetry.addData("Status", "Initialized - RED LEAVE 3 BALL");
        telemetry.update();
    }

    private void buildPaths() {
        ShootPosition0 = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shotPos))
                .setLinearHeadingInterpolation(startPos.getHeading(), shotPos.getHeading())
                .build();

        LeavePosition = follower.pathBuilder()
                .addPath(new BezierLine(shotPos, leavePos))
                .setLinearHeadingInterpolation(shotPos.getHeading(), leavePos.getHeading())
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

        double rpm1 = outtakeMotor1.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
        double rpm2 = outtakeMotor2.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
        double avgRPM = (rpm1 + rpm2) / 2.0;

        boolean shooterReady = Math.abs(avgRPM - currentTargetRPM) < 200 && currentTargetRPM > 0;

        autonomousPathUpdate(shooterReady, avgRPM);
        updateShooterPID(avgRPM);

        telemetry.addData("State", getStateDescription());
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Target RPM", "%.0f", currentTargetRPM);
        telemetry.addData("Current RPM", "%.0f", avgRPM);
        telemetry.addData("Path Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void autonomousPathUpdate(boolean shooterReady, double avgRPM) {
        boolean timedOut = pathTimer.getElapsedTime() > PATH_TIMEOUT_MS;

        switch (pathState) {
            case 0:
                if (!preloadShot) {
                    follower.followPath(ShootPosition0, true);
                    preloadShot = true;
                }
                if ((preloadShot && !follower.isBusy() && poseCloseTo(follower.getPose(), shotPos, 4.0)) || timedOut) {
                    setPathState(1);
                }
                break;

            case 1:
                if (!shootingActive) {
                    startRapidShoot();
                }
                if (continueRapidShoot(shooterReady)) {
                    setPathState(2);
                }
                break;

            case 2:
                if (!leaveStarted) {
                    follower.followPath(LeavePosition, true);
                    leaveStarted = true;
                }
                if ((leaveStarted && !follower.isBusy() && poseCloseTo(follower.getPose(), leavePos, 4.0)) || timedOut) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void startRapidShoot() {
        shootingActive = true;
        currentMode = SystemMode.SHOOT;
        currentTargetRPM = SHOOT_POWER * MAX_RPM;
        shootTimer.reset();
    }

    private boolean continueRapidShoot(boolean shooterReady) {
        if (!shootingActive) return false;

        if (shootTimer.milliseconds() < SHOOTER_SPINUP_MS) {
            return false;
        }

        if (!shooterReady && shootTimer.milliseconds() < SHOOTER_SPINUP_MS + 500) {
            return false;
        }

        if (shootTimer.milliseconds() < SHOOTER_SPINUP_MS + RAPID_SHOOT_DURATION_MS) {
            return false;
        }

        shootingActive = false;
        currentMode = SystemMode.OFF;
        currentTargetRPM = 0;
        return true;
    }

    private void applySystemMode() {
        switch (currentMode) {
            case SPITTER:
                intakeMotor.setPower(0.85);
                transferMotor.setPower(0.85);
                break;
            case INTAKE_BALLS:
                intakeMotor.setPower(-0.65);
                transferMotor.setPower(0.65);
                break;
            case SHOOT:
                intakeMotor.setPower(-0.55);
                transferMotor.setPower(-0.70);
                break;
            case OFF:
            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }

    private void updateShooterPID(double currentRPM) {
        if (currentTargetRPM <= 0) {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            shooterPID.reset();
            return;
        }

        double pidOutput = shooterPID.calculate(currentRPM, currentTargetRPM);
        double feedforward = SHOOT_POWER;
        double finalPower = feedforward + (pidOutput / MAX_RPM);
        finalPower = Math.max(0, Math.min(1.0, finalPower));

        outtakeMotor1.setPower(finalPower);
        outtakeMotor2.setPower(finalPower);
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        actionTimer.resetTimer();
    }

    private String getStateDescription() {
        switch (pathState) {
            case 0: return "MOVE TO SHOT";
            case 1: return "SHOOT PRELOAD";
            case 2: return "PARK";
            default: return "COMPLETE";
        }
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    private boolean poseCloseTo(Pose current, Pose target, double tolInches) {
        double dx = current.getX() - target.getX();
        double dy = current.getY() - target.getY();
        return Math.hypot(dx, dy) <= tolInches;
    }

    private class ShooterPIDController {
        private double kP, kI, kD;
        private double lastError = 0.0;
        private double integral = 0.0;
        private ElapsedTime timer = new ElapsedTime();

        public void setPID(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            timer.reset();
        }

        public double calculate(double current, double target) {
            double error = target - current;
            double dt = timer.seconds();

            if (dt > 0) {
                integral += error * dt;
                integral = Math.max(-1000, Math.min(1000, integral));
                double derivative = (error - lastError) / dt;
                lastError = error;
                timer.reset();
                return kP * error + kI * integral + kD * derivative;
            }
            return 0.0;
        }

        public void reset() {
            lastError = 0.0;
            integral = 0.0;
            timer.reset();
        }
    }
}