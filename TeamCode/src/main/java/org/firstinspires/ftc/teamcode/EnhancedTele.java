package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.paths.robotv2.ball12blueClose;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorShooterHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@TeleOp(name = "Enhanced TeleOp", group = "TeleOp")
public class EnhancedTele extends LinearOpMode {

    /* ================= DRIVETRAIN ================= */
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private double driveSpeed;
    private static final double LOW_SPEED = 0.4;
    private static final double HIGH_SPEED = 1.0;

    /* ================= PEDRO PATHING ================= */
    private Follower follower;
    private boolean autoPositioningActive = false;

    // Predefined positions
    private static final Pose CLOSE_SHOOT_POS = new Pose(57.655, 78.207, Math.toRadians(129));
    private static final Pose FAR_SHOOT_POS = new Pose(61.297, 16.441, Math.toRadians(112));
    private static final Pose PARK_POS = new Pose(105.324, 33.159, Math.toRadians(90));

    private static final double AUTO_DRIVE_POWER = 0.85;

    /* ================= INTAKE ================= */
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DistanceSensor intakeSensor;

    private ElapsedTime jamTimer = new ElapsedTime();
    private boolean objectDetected = false;
    private boolean jammed = false;

    private static final double JAM_DISTANCE_CM = 10.0;
    private static final double JAM_TIME = 0.2;

    /* ================= SHOOTER ================= */
    private DualMotorShooterHelper shooter;
    private boolean shooterEnabled = false;
    private DualMotorShooterHelper.ShooterMode shooterMode =
            DualMotorShooterHelper.ShooterMode.CLOSE;

    /* ================= SYSTEM MODES ================= */
    private enum SystemMode { OFF, INTAKE, SPIT, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    /* ================= LIGHTS ================= */
    private Servo shooterLight, jamLight;
    private ElapsedTime blinkTimer = new ElapsedTime();
    private boolean blinkState = false;

    private static final double LIGHT_GREEN = 0.35;
    private static final double LIGHT_RED = 0.0;
    private static final double BLINK_RATE = 0.3;

    /* ================= BUTTON DEBOUNCING ================= */
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastCircle = false;
    private boolean lastSquare = false;

    @Override
    public void runOpMode() {

        initHardware();
        initPedroPathing();

        telemetry.addLine("READY - Enhanced TeleOp");
        telemetry.addLine("Gamepad2 D-Pad Up: Close Shoot Position");
        telemetry.addLine("Gamepad2 D-Pad Down: Far Shoot Position");
        telemetry.addLine("Gamepad2 D-Pad Right: Park Position");
        telemetry.addLine("Gamepad2 D-Pad Left: Cancel Auto-Position");
        telemetry.addLine("Gamepad2 Circle: Close Shooter Mode");
        telemetry.addLine("Gamepad2 Square: Far Shooter Mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            updateJamDetection();
            handleAutoPositioning();
            handleModeSelection();
            handleDrive();
            handleShooter();
            applyModeLogic();
            updateLights();

            shooter.update();
            if (follower != null) {
                follower.update();
            }

            telemetryOutput();

            sleep(20);
        }

        stopAll();
    }

    /* ================= INIT ================= */

    private void initHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        shooter = new DualMotorShooterHelper(
                hardwareMap,
                Constants.shooterCoefficients
        );

        shooterLight = hardwareMap.get(Servo.class, "shooter_light");
        jamLight = hardwareMap.get(Servo.class, "jam_light");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight, intakeMotor, transferMotor
        };
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void initPedroPathing() {
        try {
            follower = Constants.createFollower(hardwareMap);

            // Load pose from autonomous
            follower.setStartingPose(ball12blueClose.autoEndPose);

            telemetry.addLine("Pedro Pathing Initialized");
            telemetry.addData("Auto End Pose X", ball12blueClose.autoEndPose.getX());
            telemetry.addData("Auto End Pose Y", ball12blueClose.autoEndPose.getY());
            telemetry.addData("Auto End Heading", Math.toDegrees(ball12blueClose.autoEndPose.getHeading()));
        } catch (Exception e) {
            telemetry.addLine("Pedro Pathing Init Failed: " + e.getMessage());
            follower = null;
        }
    }

    /* ================= AUTO POSITIONING ================= */

    private void handleAutoPositioning() {
        if (follower == null) return;

        // Gamepad2 D-Pad controls
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        boolean dpadRight = gamepad2.dpad_right;
        boolean dpadLeft = gamepad2.dpad_left;

        // Close shoot position (D-Pad Up)
        if (dpadUp && !lastDpadUp && !autoPositioningActive) {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            CLOSE_SHOOT_POS
                    ))
                    .setLinearHeadingInterpolation(
                            follower.getPose().getHeading(),
                            CLOSE_SHOOT_POS.getHeading()
                    )
                    .build();

            follower.followPath(path);
            follower.setMaxPower(AUTO_DRIVE_POWER);
            autoPositioningActive = true;
        }

        // Far shoot position (D-Pad Down)
        if (dpadDown && !lastDpadDown && !autoPositioningActive) {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            FAR_SHOOT_POS
                    ))
                    .setLinearHeadingInterpolation(
                            follower.getPose().getHeading(),
                            FAR_SHOOT_POS.getHeading()
                    )
                    .build();

            follower.followPath(path);
            follower.setMaxPower(AUTO_DRIVE_POWER);
            autoPositioningActive = true;
        }

        // Park position (D-Pad Right)
        if (dpadRight && !lastDpadRight && !autoPositioningActive) {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(
                            follower.getPose(),
                            PARK_POS
                    ))
                    .setLinearHeadingInterpolation(
                            follower.getPose().getHeading(),
                            PARK_POS.getHeading()
                    )
                    .build();

            follower.followPath(path);
            follower.setMaxPower(AUTO_DRIVE_POWER);
            autoPositioningActive = true;
        }

        // Check for manual input to cancel auto-positioning
        boolean manualInput = Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1;

        // Cancel auto-positioning (D-Pad Left OR manual input)
        if ((dpadLeft && !lastDpadLeft && autoPositioningActive) ||
                (autoPositioningActive && manualInput)) {
            follower.breakFollowing();
            autoPositioningActive = false;
        }

        // Check if auto-positioning completed
        if (autoPositioningActive && !follower.isBusy()) {
            autoPositioningActive = false;
        }

        // Update button states
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadRight = dpadRight;
        lastDpadLeft = dpadLeft;
    }

    /* ================= JAM ================= */

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
            jamTimer.reset();
        }
    }

    /* ================= MODE SELECT ================= */

    private void handleModeSelection() {
        if (gamepad1.cross) currentMode = SystemMode.INTAKE;
        if (gamepad1.triangle) currentMode = SystemMode.SPIT;
        if (gamepad1.circle) currentMode = SystemMode.SHOOT;
        if (gamepad1.square) currentMode = SystemMode.OFF;
    }

    /* ================= DRIVE ================= */

    private void handleDrive() {

        // If auto-positioning is active, don't accept manual drive input
        if (autoPositioningActive) {
            return;
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        driveSpeed = gamepad1.left_stick_button ? LOW_SPEED : HIGH_SPEED;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX *= driveSpeed;
        rotY *= driveSpeed;
        rx *= driveSpeed;

        double fl = rotY + rotX + rx;
        double bl = rotY - rotX + rx;
        double fr = rotY - rotX - rx;
        double br = rotY + rotX - rx;

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        frontLeft.setPower(fl / denom);
        backLeft.setPower(bl / denom);
        frontRight.setPower(fr / denom);
        backRight.setPower(br / denom);
    }

    /* ================= MODE LOGIC ================= */

    private void applyModeLogic() {
        switch (currentMode) {

            case INTAKE:
                if (jammed) {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(0);
                } else {
                    intakeMotor.setPower(-1.0);
                    transferMotor.setPower(-0.15);
                }
                break;

            case SPIT:
                intakeMotor.setPower(0.9);
                transferMotor.setPower(-0.85);
                break;

            case SHOOT:
                intakeMotor.setPower(-0.55);
                transferMotor.setPower(0.9);
                break;

            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
        }
    }

    /* ================= SHOOTER ================= */

    private void handleShooter() {

        // Shooter mode selection - Gamepad2 Circle/Square
        boolean circle = gamepad2.circle;
        boolean square = gamepad2.square;

        if (circle && !lastCircle) {
            shooterMode = DualMotorShooterHelper.ShooterMode.CLOSE;
        }
        if (square && !lastSquare) {
            shooterMode = DualMotorShooterHelper.ShooterMode.FAR;
        }

        lastCircle = circle;
        lastSquare = square;

        if (gamepad1.right_bumper) shooterEnabled = true;
        if (gamepad1.left_bumper) shooterEnabled = false;

        if (shooterEnabled) {
            if (shooterMode == DualMotorShooterHelper.ShooterMode.CLOSE)
                shooter.runCloseShot();
            else
                shooter.runFarShot();
        } else {
            shooter.stop();
        }
    }

    /* ================= LIGHTS ================= */

    private void updateLights() {

        if (shooterEnabled) {
            if (shooter.isAtTargetVelocity()) {
                if (blinkTimer.seconds() > BLINK_RATE) {
                    blinkState = !blinkState;
                    blinkTimer.reset();
                }
                shooterLight.setPosition(blinkState ? LIGHT_GREEN : 1.0);
            } else {
                shooterLight.setPosition(LIGHT_GREEN);
            }
        } else {
            shooterLight.setPosition(LIGHT_RED);
        }

        jamLight.setPosition(jammed ? LIGHT_RED : LIGHT_GREEN);
    }

    /* ================= TELEMETRY ================= */

    private void telemetryOutput() {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Jammed", jammed);
        telemetry.addData("Shooter", shooterEnabled ? shooterMode : "OFF");
        telemetry.addData("Auto Position", autoPositioningActive ? "ACTIVE" : "Manual");

        if (follower != null) {
            Pose pose = follower.getPose();
            telemetry.addData("X", "%.1f", pose.getX());
            telemetry.addData("Y", "%.1f", pose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        }

        telemetry.update();
    }

    /* ================= STOP ================= */

    private void stopAll() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        shooter.stop();
    }
}