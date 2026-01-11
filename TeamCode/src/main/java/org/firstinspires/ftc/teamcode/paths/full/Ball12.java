package org.firstinspires.ftc.teamcode.paths.full;

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

@Autonomous(name = "12 Ball BLUE - Front", group = "Auto")
public class Ball12 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx outtakeMotor1, outtakeMotor2;
    private IMU imu;

    private ShooterPIDController shooterPID;

    private int pathState = 0;
    private double speed = 1.0;

    private static final double PATH_TIMEOUT_MS = 3500;

    private enum SystemMode { OFF, SPITTER, INTAKE_BALLS, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    private static final double SHOOT_POWER = 0.45;
    private static final double MAX_RPM = 5800.0;
    private final int RAPID_SHOOT_DURATION_MS = 1500;  // 1.5 seconds rapid fire

    private boolean shootingActive = false;
    private ElapsedTime shootTimer = new ElapsedTime();
    private double currentTargetRPM = 0;

    // Path completion flags
    private boolean shoot0Started = false, shoot0Complete = false;
    private boolean intakePos1Started = false, intakePos1Complete = false;
    private boolean intakeBalls1Started = false, intakeBalls1Complete = false;
    private boolean shoot1Started = false, shoot1Complete = false;
    private boolean intakePos2Started = false, intakePos2Complete = false;
    private boolean intakeBalls2Started = false, intakeBalls2Complete = false;
    private boolean shootPos2Started = false, shootPos2Complete = false;
    private boolean intakePos3Started = false, intakePos3Complete = false;
    private boolean intakeBalls3Started = false, intakeBalls3Complete = false;
    private boolean shootPos3Started = false, shootPos3Complete = false;
    private boolean leaveStarted = false;

    // Waypoints
    private static final Pose startPos = new Pose(26.814, 129.434, Math.toRadians(143));
    private static final Pose shotPos = new Pose(54.000, 101.000, Math.toRadians(138));
    private static final Pose intakePos1Start = new Pose(44.193, 83.752, Math.toRadians(0));
    private static final Pose intakePos1End = new Pose(19.200, 83.421, Math.toRadians(0));
    private static final Pose intakePos2Start = new Pose(47.503, 60.248, Math.toRadians(0));
    private static final Pose intakePos2End = new Pose(12.745, 56.110, Math.toRadians(0));
    private static final Pose intakePos3Start = new Pose(54.455, 35.421, Math.toRadians(0));
    private static final Pose intakePos3End = new Pose(15.062, 35.421, Math.toRadians(0));
    private static final Pose leavePos = new Pose(48.900, 66.600, Math.toRadians(0));

    private PathChain Shoot0, IntakePos1, IntakeBalls1, Shoot1;
    private PathChain IntakePos2, IntakeBalls2, ShootPos2;
    private PathChain IntakePos3, IntakeBalls3, ShootPos3, LeavePos;

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

        telemetry.addData("Status", "Initialized - BLUE 12 BALL");
        telemetry.update();
    }

    private void buildPaths() {
        Shoot0 = follower.pathBuilder()
                .addPath(new BezierLine(startPos, shotPos))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(138))
                .build();

        IntakePos1 = follower.pathBuilder()
                .addPath(new BezierCurve(shotPos, new Pose(70.345, 83.090, 0), intakePos1Start))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        IntakeBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePos1Start, intakePos1End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePos1End, shotPos))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(138))
                .build();

        IntakePos2 = follower.pathBuilder()
                .addPath(new BezierCurve(shotPos, new Pose(69.352, 61.076, 0), intakePos2Start))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        IntakeBalls2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePos2Start, new Pose(20.690, 61.738, 0), intakePos2End))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        ShootPos2 = follower.pathBuilder()
                .addPath(new BezierCurve(intakePos2End, new Pose(70.345, 85.076, 0), shotPos))
                .setTangentHeadingInterpolation()
                .build();

        IntakePos3 = follower.pathBuilder()
                .addPath(new BezierLine(shotPos, intakePos3Start))
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(0))
                .build();

        IntakeBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePos3Start, intakePos3End))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePos3End, shotPos))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(138))
                .build();

        LeavePos = follower.pathBuilder()
                .addPath(new BezierLine(shotPos, leavePos))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(0))
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

        autonomousPathUpdate(avgRPM);
        updateShooterPID(avgRPM);

        telemetry.addData("State", getStateDescription());
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Target RPM", "%.0f", currentTargetRPM);
        telemetry.addData("Current RPM", "%.0f", avgRPM);
        telemetry.addData("Path Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    private void autonomousPathUpdate(double avgRPM) {
        boolean timedOut = pathTimer.getElapsedTime() > PATH_TIMEOUT_MS;

        switch (pathState) {
            case 0: // Move to shoot position (outtake spins up during path)
                if (!shoot0Started) {
                    currentTargetRPM = SHOOT_POWER * MAX_RPM;
                    follower.followPath(Shoot0, true);
                    shoot0Started = true;
                }
                if ((shoot0Started && !shoot0Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), shotPos, 2.0)) || timedOut) {
                    shoot0Complete = true;
                    setPathState(1);
                }
                break;

            case 1: // Fire preload (2 seconds)
                if (!shootingActive) {
                    startRapidShoot();
                }
                if (continueRapidShoot()) {
                    setPathState(2);
                }
                break;

            case 2: // Go to intake position 1
                if (!intakePos1Started) {
                    follower.followPath(IntakePos1, true);
                    intakePos1Started = true;
                }
                if ((intakePos1Started && !intakePos1Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos1Start, 2.0)) || timedOut) {
                    intakePos1Complete = true;
                    setPathState(3);
                }
                break;

            case 3: // Intake balls 1
                if (!intakeBalls1Started) {
                    currentMode = SystemMode.INTAKE_BALLS;
                    speed = 0.85;
                    follower.followPath(IntakeBalls1, true);
                    intakeBalls1Started = true;
                }
                if ((intakeBalls1Started && !intakeBalls1Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos1End, 2.0)) || timedOut) {
                    currentMode = SystemMode.OFF;
                    speed = 1.0;
                    intakeBalls1Complete = true;
                    setPathState(4);
                }
                break;

            case 4: // Return to shoot position 1 (outtake spins up during path)
                if (!shoot1Started) {
                    currentTargetRPM = SHOOT_POWER * MAX_RPM;
                    follower.followPath(Shoot1, true);
                    shoot1Started = true;
                }
                if ((shoot1Started && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), shotPos, 2.0)) || timedOut) {
                    setPathState(5);
                }
                break;

            case 5: // Fire balls 1 (2 seconds)
                if (!shootingActive) {
                    startRapidShoot();
                }
                if (continueRapidShoot()) {
                    setPathState(6);
                }
                break;

            case 6: // Go to intake position 2
                if (!intakePos2Started) {
                    follower.followPath(IntakePos2, true);
                    intakePos2Started = true;
                }
                if ((intakePos2Started && !intakePos2Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos2Start, 2.0)) || timedOut) {
                    intakePos2Complete = true;
                    setPathState(7);
                }
                break;

            case 7: // Intake balls 2
                if (!intakeBalls2Started) {
                    currentMode = SystemMode.INTAKE_BALLS;
                    speed = 0.85;
                    follower.followPath(IntakeBalls2, true);
                    intakeBalls2Started = true;
                }
                if ((intakeBalls2Started && !intakeBalls2Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos2End, 2.0)) || timedOut) {
                    currentMode = SystemMode.OFF;
                    speed = 1.0;
                    intakeBalls2Complete = true;
                    setPathState(8);
                }
                break;

            case 8: // Return to shoot position 2 (outtake spins up during path)
                if (!shootPos2Started) {
                    currentTargetRPM = SHOOT_POWER * MAX_RPM;
                    follower.followPath(ShootPos2, true);
                    shootPos2Started = true;
                }
                if ((shootPos2Started && !shootPos2Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), shotPos, 2.0)) || timedOut) {
                    shootPos2Complete = true;
                    setPathState(9);
                }
                break;

            case 9: // Fire balls 2 (2 seconds)
                if (!shootingActive) {
                    startRapidShoot();
                }
                if (continueRapidShoot()) {
                    setPathState(10);
                }
                break;

            case 10: // Go to intake position 3
                if (!intakePos3Started) {
                    follower.followPath(IntakePos3, true);
                    intakePos3Started = true;
                }
                if ((intakePos3Started && !intakePos3Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos3Start, 2.0)) || timedOut) {
                    intakePos3Complete = true;
                    setPathState(11);
                }
                break;

            case 11: // Intake balls 3
                if (!intakeBalls3Started) {
                    currentMode = SystemMode.INTAKE_BALLS;
                    speed = 0.85;
                    follower.followPath(IntakeBalls3, true);
                    intakeBalls3Started = true;
                }
                if ((intakeBalls3Started && !intakeBalls3Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), intakePos3End, 2.0)) || timedOut) {
                    currentMode = SystemMode.OFF;
                    speed = 1.0;
                    intakeBalls3Complete = true;
                    setPathState(12);
                }
                break;

            case 12: // Return to shoot position 3 (outtake spins up during path)
                if (!shootPos3Started) {
                    currentTargetRPM = SHOOT_POWER * MAX_RPM;
                    follower.followPath(ShootPos3, true);
                    shootPos3Started = true;
                }
                if ((shootPos3Started && !shootPos3Complete && !follower.isBusy()
                        && poseCloseTo(follower.getPose(), shotPos, 2.0)) || timedOut) {
                    shootPos3Complete = true;
                    setPathState(13);
                }
                break;

            case 13: // Fire balls 3 (2 seconds)
                if (!shootingActive) {
                    startRapidShoot();
                }
                if (continueRapidShoot()) {
                    setPathState(14);
                }
                break;

            case 14: // Park
                if (!leaveStarted) {
                    follower.followPath(LeavePos, true);
                    leaveStarted = true;
                }
                if ((leaveStarted && !follower.isBusy() && poseCloseTo(follower.getPose(), leavePos, 2.0)) || timedOut) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void startRapidShoot() {
        shootingActive = true;
        currentMode = SystemMode.SHOOT;
        shootTimer.reset();
    }

    private boolean continueRapidShoot() {
        if (!shootingActive) return false;

        if (shootTimer.milliseconds() < RAPID_SHOOT_DURATION_MS) {
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
                intakeMotor.setPower(-0.75);
                transferMotor.setPower(0.65);
                break;
            case SHOOT:
                intakeMotor.setPower(-0.85);
                transferMotor.setPower(-1.0);
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
            case 2: return "GO TO INTAKE 1";
            case 3: return "INTAKE BALLS 1";
            case 4: return "RETURN TO SHOOT 1";
            case 5: return "SHOOT 1";
            case 6: return "GO TO INTAKE 2";
            case 7: return "INTAKE BALLS 2";
            case 8: return "RETURN TO SHOOT 2";
            case 9: return "SHOOT 2";
            case 10: return "GO TO INTAKE 3";
            case 11: return "INTAKE BALLS 3";
            case 12: return "RETURN TO SHOOT 3";
            case 13: return "SHOOT 3";
            case 14: return "PARK";
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