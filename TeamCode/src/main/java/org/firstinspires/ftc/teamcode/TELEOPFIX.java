package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FilteredLimelightHelper;
import org.firstinspires.ftc.teamcode.subsystems.DistanceHelper;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeAutoPower;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.HashMap;
import java.io.File;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOP RapidFire main", group="TeleOp")
public class TELEOPFIX extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor, transferMotor;
    private DcMotorEx outtakeMotor1, outtakeMotor2;
    private IMU imu;

    private FilteredLimelightHelper limelight;
    private DistanceHelper distanceHelper;
    private OuttakeAutoPower outtakeAuto;

    private Follower follower;
    private boolean automatedDrive = false;
    private enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;

    private ElapsedTime runtime = new ElapsedTime();

    private double driveSpeed = 1.0;  // CHANGED: Full power by default

    // SYSTEM MODES
    private enum SystemMode { OFF, SPITTER, INTAKE_BALLS, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    private boolean outtakeEnabled = false;
    private boolean manualOuttakeMode = false;
    private double outtakeManualPower = 0.3;

    private boolean lastLeftBumper1 = false;
    private boolean lastRightBumper1 = false;
    private boolean lastLeftBumper2 = false;
    private boolean lastRightBumper2 = false;

    // Acceleration smoothing - DISABLED for instant response
    private double targetFL, targetFR, targetBL, targetBR;
    private double smoothFL, smoothFR, smoothBL, smoothBR;
    private final double ACCELERATION = 1.0;  // CHANGED: Instant acceleration (no smoothing)

    private final double ALIGN_SPEED = 0.04;
    private final double ALIGN_DEADZONE = 3.0;

    // Auto start type selection
    private enum AutoStartType { FRONT, BACK, LEAVE }
    private AutoStartType autoStartType = AutoStartType.FRONT;

    // BLUE Alliance Positions (base positions)
    private final Pose BLUE_PARK = new Pose(105.103, 32.938, Math.toRadians(90));
    private final Pose BLUE_SHOOT = new Pose(57.766, 86.731, Math.toRadians(134));
    private final Pose BLUE_START_FRONT = new Pose(48.9, 66.6, Math.toRadians(0));
    private final Pose BLUE_START_BACK = new Pose(36.248, 11.586, Math.toRadians(0));
    private final Pose BLUE_START_LEAVE = new Pose(35.752, 13.903, Math.toRadians(0));

    // RED Alliance Positions (mirrored from BLUE)
    private final Pose RED_PARK = new Pose(105.103, 32.938, Math.toRadians(90)).mirror();
    private final Pose RED_SHOOT = new Pose(57.766, 86.731, Math.toRadians(134)).mirror();
    private final Pose RED_START_FRONT = new Pose(48.9, 66.6, Math.toRadians(0)).mirror();
    private final Pose RED_START_BACK = new Pose(36.248, 11.586, Math.toRadians(0)).mirror();
    private final Pose RED_START_LEAVE = new Pose(35.752, 13.903, Math.toRadians(0)).mirror();

    // MACHINE LEARNING SHOT MAP
    private HashMap<Double, Double> shotPowerMap = new HashMap<>();
    private final double DIST_STEP = 0.2;
    private final double DEFAULT_POWER = 0.50;
    private File saveFile;

    // PID CONTROLLER
    private ShooterPIDController pidController;
    private static final double P = 0.008;
    private static final double I = 0.0003;
    private static final double D = 0.0001;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double MAX_RPM = 5800.0;

    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // --- INIT HARDWARE ---
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize subsystems
        limelight = new FilteredLimelightHelper(hardwareMap);
        distanceHelper = new DistanceHelper(limelight);
        outtakeAuto = new OuttakeAutoPower(outtakeMotor1, outtakeMotor2, distanceHelper);

        // DON'T initialize follower yet - wait until after alliance selection

        pidController = new ShooterPIDController();
        pidController.setPID(P, I, D);
        pidTimer.reset();

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Setup outtake motors for velocity control with encoders
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor[] allMotors = {frontLeft, frontRight, backLeft, backRight, intakeMotor, transferMotor, outtakeMotor1, outtakeMotor2};
        for (DcMotor m : allMotors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        saveFile = new File(hardwareMap.appContext.getFilesDir(), "shotPowers.txt");
        loadShotPowers();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== ALLIANCE SELECTION ===");
        telemetry.addLine("DPad Left: RED | DPad Right: BLUE");
        telemetry.addLine("");
        telemetry.addLine("=== AUTO START TYPE ===");
        telemetry.addLine("Triangle: FRONT (9 Ball)");
        telemetry.addLine("Cross: BACK (6 Ball)");
        telemetry.addLine("Circle: LEAVE (3 Ball)");
        telemetry.addLine("");
        telemetry.addLine("IMPORTANT: Select the SAME alliance");
        telemetry.addLine("and AUTO TYPE as your autonomous!");
        telemetry.update();

        // ALLIANCE AND AUTO TYPE SELECTION LOOP
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                currentAlliance = Alliance.RED;
                sleep(200);
            }
            if (gamepad1.dpad_right) {
                currentAlliance = Alliance.BLUE;
                sleep(200);
            }
            if (gamepad1.triangle) {
                autoStartType = AutoStartType.FRONT;
                sleep(200);
            }
            if (gamepad1.cross) {
                autoStartType = AutoStartType.BACK;
                sleep(200);
            }
            if (gamepad1.circle) {
                autoStartType = AutoStartType.LEAVE;
                sleep(200);
            }

            telemetry.addData("Alliance", currentAlliance);
            telemetry.addData("Auto Start", autoStartType);
            telemetry.addLine("Press START when ready");
            telemetry.update();
        }

        // NOW initialize follower with correct starting pose based on alliance AND auto type
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        // Set starting pose to match where autonomous left the robot
        Pose startPose;
        if (currentAlliance == Alliance.RED) {
            if (autoStartType == AutoStartType.FRONT) {
                startPose = RED_START_FRONT;
            } else if (autoStartType == AutoStartType.BACK) {
                startPose = RED_START_BACK;
            } else {
                startPose = RED_START_LEAVE;
            }
            telemetry.addData("Starting Pose", "RED " + autoStartType + ": " + startPose.toString());
        } else {
            if (autoStartType == AutoStartType.FRONT) {
                startPose = BLUE_START_FRONT;
            } else if (autoStartType == AutoStartType.BACK) {
                startPose = BLUE_START_BACK;
            } else {
                startPose = BLUE_START_LEAVE;
            }
            telemetry.addData("Starting Pose", "BLUE " + autoStartType + ": " + startPose.toString());
        }
        follower.setStartingPose(startPose);
        telemetry.update();
        sleep(500);

        waitForStart();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();

            // === DRIVER 1: DRIVETRAIN ===
            boolean inAuto = automatedDrive || follower.isBusy();

            if (!inAuto) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= driveSpeed;
                rotY *= driveSpeed;
                rx *= driveSpeed;

                // Auto-align assist
                if (gamepad1.left_trigger > 0.3) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        // IF LIMELIGHT IS ROTATED 90° CW, USE THIS:
                        double tx = -result.getTy();
                        // Normal orientation: double tx = result.getTx();

                        if (Math.abs(tx) > ALIGN_DEADZONE) {
                            double alignCorrection = Math.max(Math.min(tx * ALIGN_SPEED, 0.5), -0.5);
                            rx += alignCorrection;
                        }
                    }
                }

                targetFL = rotY + rotX + rx;
                targetBL = rotY - rotX + rx;
                targetFR = rotY - rotX - rx;
                targetBR = rotY + rotX - rx;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                targetFL /= denominator;
                targetBL /= denominator;
                targetFR /= denominator;
                targetBR /= denominator;

                // Smoothing with acceleration
                smoothFL += Math.signum(targetFL - smoothFL) * Math.min(Math.abs(targetFL - smoothFL), ACCELERATION);
                smoothBL += Math.signum(targetBL - smoothBL) * Math.min(Math.abs(targetBL - smoothBL), ACCELERATION);
                smoothFR += Math.signum(targetFR - smoothFR) * Math.min(Math.abs(targetFR - smoothFR), ACCELERATION);
                smoothBR += Math.signum(targetBR - smoothBR) * Math.min(Math.abs(targetBR - smoothBR), ACCELERATION);

                frontLeft.setPower(smoothFL);
                backLeft.setPower(smoothBL);
                frontRight.setPower(smoothFR);
                backRight.setPower(smoothBR);
            }

            // DRIVER 1: Speed adjustment
            boolean lb1 = gamepad1.left_bumper;
            boolean rb1 = gamepad1.right_bumper;
            if (lb1 && !lastLeftBumper1) driveSpeed = Math.max(driveSpeed - 0.1, 0.1);
            if (rb1 && !lastRightBumper1) driveSpeed = Math.min(driveSpeed + 0.1, 1.0);
            lastLeftBumper1 = lb1;
            lastRightBumper1 = rb1;

            // DRIVER 1: Reset IMU
            if (gamepad1.square) {
                imu.resetYaw();
                sleep(100);
            }

            // DRIVER 1: Auto shoot position (alliance-specific)
            if (gamepad1.dpad_down) {
                Pose shootPose = (currentAlliance == Alliance.RED) ? RED_SHOOT : BLUE_SHOOT;
                PathChain shootPath = follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower.getPose(), shootPose)))
                        .setConstantHeadingInterpolation(shootPose.getHeading())
                        .build();
                follower.followPath(shootPath);
                automatedDrive = true;
                sleep(200);
            }

            // DRIVER 1: Auto park (alliance-specific)
            if (gamepad1.dpad_up) {
                Pose parkPose = (currentAlliance == Alliance.RED) ? RED_PARK : BLUE_PARK;
                PathChain parkPath = follower.pathBuilder()
                        .addPath(new Path(new BezierLine(follower.getPose(), parkPose)))
                        .setConstantHeadingInterpolation(parkPose.getHeading())
                        .build();
                follower.followPath(parkPath);
                automatedDrive = true;
                sleep(200);
            }

            // Stop automated drive if done or cancelled
            if (automatedDrive && (!follower.isBusy() || gamepad1.circle)) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

            // === DRIVER 2: SYSTEM MODES ===
            if (gamepad2.triangle) {
                currentMode = SystemMode.SPITTER;
                sleep(200);
            }

            if (gamepad2.cross) {
                currentMode = SystemMode.INTAKE_BALLS;
                sleep(200);
            }

            if (gamepad2.circle) {
                currentMode = SystemMode.SHOOT;
                sleep(200);
            }

            if (gamepad2.square) {
                currentMode = SystemMode.OFF;
                sleep(200);
            }

            // Apply mode-specific powers
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

            // === DRIVER 2: OUTTAKE WITH ML/DISTANCE ===

            // Get distance from DistanceHelper
            Double dist = distanceHelper.getDistanceMeters();
            double keyDist = (dist != null) ? Math.round(dist / DIST_STEP) * DIST_STEP : 0.0;

            // DPAD: Fine-tune learned power at current distance
            if (gamepad2.dpad_left) {
                double currentPower = shotPowerMap.getOrDefault(keyDist, DEFAULT_POWER);
                shotPowerMap.put(keyDist, Math.max(currentPower - 0.01, 0.0));
                sleep(150);
            }
            if (gamepad2.dpad_right) {
                double currentPower = shotPowerMap.getOrDefault(keyDist, DEFAULT_POWER);
                shotPowerMap.put(keyDist, Math.min(currentPower + 0.01, 1.0));
                sleep(150);
            }

            // Bumpers: Adjust manual power
            boolean lb2 = gamepad2.left_bumper;
            boolean rb2 = gamepad2.right_bumper;
            if (lb2 && !lastLeftBumper2) outtakeManualPower = Math.max(outtakeManualPower - 0.02, 0);
            if (rb2 && !lastRightBumper2) outtakeManualPower = Math.min(outtakeManualPower + 0.02, 1.0);
            lastLeftBumper2 = lb2;
            lastRightBumper2 = rb2;

            // L2: Turn OFF outtake (OVERRIDES EVERYTHING)
            // R2: MANUAL MODE (uses outtakeManualPower)
            // Neither: AUTO MODE (uses ML distance-based power)
            if (gamepad2.left_trigger > 0.5) {
                outtakeEnabled = false;
                manualOuttakeMode = false;
            } else if (gamepad2.right_trigger > 0.1) {
                outtakeEnabled = true;
                manualOuttakeMode = true;
            } else if (outtakeEnabled) {
                manualOuttakeMode = false;
            }

            // Determine target power
            double autoPower = shotPowerMap.getOrDefault(keyDist, DEFAULT_POWER);
            double desiredPower = manualOuttakeMode ? outtakeManualPower : autoPower;

            // Apply PID control
            if (outtakeEnabled) {
                double targetRPM = desiredPower * MAX_RPM;

                // Get velocities from encoders
                double velocity1 = outtakeMotor1.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
                double velocity2 = outtakeMotor2.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
                double avgVelocity = (velocity1 + velocity2) / 2.0;

                double pidOutput = pidController.calculate(avgVelocity, targetRPM);

                // Feedforward + PID correction
                double feedforward = desiredPower;
                double finalPower = feedforward + (pidOutput / MAX_RPM);
                finalPower = Math.max(0, Math.min(1.0, finalPower));

                outtakeMotor1.setPower(finalPower);
                outtakeMotor2.setPower(finalPower);
            } else {
                outtakeMotor1.setPower(0);
                outtakeMotor2.setPower(0);
                pidController.reset();
            }

            // === TELEMETRY ===
            double velocity1 = outtakeMotor1.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
            double velocity2 = outtakeMotor2.getVelocity() * (60.0 / TICKS_PER_REV) / GEAR_RATIO;
            double avgVelocity = (velocity1 + velocity2) / 2.0;
            double targetRPM = desiredPower * MAX_RPM;
            double rpmError = Math.abs(targetRPM - avgVelocity);

            // Check if limelight sees a tag
            boolean tagVisible = limelight.hasDetection();
            int detectedTag = limelight.getDetectedTagId();

            telemetry.addLine("╔════ DRIVER 1 ════╗");
            telemetry.addData("│ Speed", "%.0f%%", driveSpeed * 100);
            telemetry.addData("│ Auto Drive", automatedDrive ? "ACTIVE" : "Manual");
            telemetry.addData("│ Alliance", currentAlliance);
            telemetry.addData("│ Auto-Align", (gamepad1.left_trigger > 0.3) ? "ACTIVE" : "OFF");
            telemetry.addData("│ Position", "X:%.1f Y:%.1f H:%.0f°",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addLine("╠════ DRIVER 2 ════╣");
            telemetry.addData("│ Mode", currentMode);
            telemetry.addData("│ Outtake", outtakeEnabled ? (manualOuttakeMode ? "MANUAL" : "AUTO") : "OFF");

            telemetry.addLine("╠═══ ML SYSTEM ════╣");
            telemetry.addData("│ Tag Visible", tagVisible ? "YES (ID:" + detectedTag + ")" : "NO");
            telemetry.addData("│ Distance", dist != null ? String.format("%.2fm", dist) : "N/A");
            telemetry.addData("│ ML Key", String.format("%.1fm", keyDist));
            telemetry.addData("│ ML Power", "%.3f", autoPower);
            telemetry.addData("│ Manual Pwr", "%.3f", outtakeManualPower);
            telemetry.addData("│ Active Pwr", "%.3f", desiredPower);

            telemetry.addLine("╠═══ PID INFO ═════╣");
            telemetry.addData("│ Target RPM", "%.0f", targetRPM);
            telemetry.addData("│ Current RPM", "%.0f", avgVelocity);
            telemetry.addData("│ RPM Error", "%.0f", rpmError);
            telemetry.addLine("╚══════════════════╝");

            telemetry.update();
        }

        saveShotPowers();
        for (DcMotor m : allMotors) m.setPower(0);
        limelight.close();
    }

    // === FILE I/O ===
    private void saveShotPowers() {
        try (FileWriter writer = new FileWriter(saveFile, false)) {
            for (Double key : shotPowerMap.keySet()) {
                writer.write(key + "," + shotPowerMap.get(key) + "\n");
            }
        } catch (IOException e) {
            telemetry.addData("Error", "Could not save shot powers");
        }
    }

    private void loadShotPowers() {
        if (!saveFile.exists()) return;
        try (BufferedReader reader = new BufferedReader(new FileReader(saveFile))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    shotPowerMap.put(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]));
                }
            }
        } catch (IOException e) {
            telemetry.addData("Error", "Could not load shot powers");
        }
    }

    // === PID CONTROLLER ===
    private class ShooterPIDController {
        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;

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

                // Anti-windup
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