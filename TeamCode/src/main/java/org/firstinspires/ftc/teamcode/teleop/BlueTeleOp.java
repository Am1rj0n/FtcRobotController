package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Blue Turret Tele", group = "Scrap")
public class BlueTeleOp extends OpMode {

    private static final boolean IS_RED = false;

    // GM0 field-centric - direct motor control (same as SafeTeleOp)
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU     imu;
    private double  fieldCentricOffset = 0.0;

    private static final boolean FIELD_CENTRIC  = true;
    private double speedMultiplier = 1.0;
    private static final double MIN_SPEED       = 0.3;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    // Pedro for odometry, turret, SWM only
    private Drivetrain          drivetrain;
    private Intake              intake;
    private Shooter             shooter;
    private Turret              turret;
    private Limelight           limelight;
    private ShootingWhileMoving swm;
    private Follower            follower;

    private final ElapsedTime runtime = new ElapsedTime();

    private boolean lastSquare    = false;
    private boolean lastCross     = false;
    private boolean lastTriangle  = false;
    private boolean lastCircle    = false;
    private boolean lastL1        = false;
    private boolean lastR1        = false;
    private boolean lastL2        = false;
    private boolean lastR2        = false;
    private boolean lastR3        = false;
    private boolean lastTouchpad  = false;
    private boolean lastOptions   = false;
    private boolean lastDpadDown  = false;
    private boolean lastDpadRight = false;
    private boolean lastGP2L1     = false;
    private boolean lastGP2R1     = false;
    private boolean lastGP2DpadUp = false;

    @Override
    public void init() {
        // GM0 direct motor control
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        // Pedro for odometry + turret + SWM only
        follower   = Constants.createFollower(hardwareMap);
        limelight  = new Limelight(hardwareMap, IS_RED);
        drivetrain = new Drivetrain(hardwareMap, follower, IS_RED);
        intake     = new Intake(hardwareMap);
        shooter    = new Shooter(hardwareMap);
        turret     = new Turret(hardwareMap, limelight, IS_RED);
        swm        = new ShootingWhileMoving(follower, shooter, turret, IS_RED);

        telemetry.addData("Auto Pose Available", AutoToTeleTransfer.finalPose != null);
        telemetry.update();
    }

    @Override
    public void start() {
        if (AutoToTeleTransfer.finalPose != null) {
            follower.setStartingPose(AutoToTeleTransfer.finalPose);
            gamepad1.rumble(500);
        } else {
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        }
        follower.startTeleopDrive();

        imu.resetYaw();
        fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        limelight.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        swm.update();
        limelight.updateMegaTag2Orientation(follower);

        handleDrive();
        handleIntake();
        handleShooter();
        handleTurret();
        handleSWM();
        handleLocalization();

        if (shooter.isActive()) {
            shooter.setRPMForDistance(swm.getDistanceForRPM() * 0.0254);
        }

        turret.update(follower.getPose());
        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        limelight.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.left_bumper && !lastL1)
            speedMultiplier = Math.max(MIN_SPEED, speedMultiplier - SPEED_INCREMENT);
        if (gamepad1.right_bumper && !lastR1)
            speedMultiplier = Math.min(MAX_SPEED, speedMultiplier + SPEED_INCREMENT);
        lastL1 = gamepad1.left_bumper;
        lastR1 = gamepad1.right_bumper;

        if (gamepad1.right_stick_button && !lastR3) {
            fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            if (drivetrain.isHolding()) gamepad1.rumble(500);
        }
        lastTouchpad = gamepad1.touchpad;

        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        double rotX, rotY;
        if (!FIELD_CENTRIC) {
            rotX = x;
            rotY = y;
        } else {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset;
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }

        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower( (rotY + rotX + rx) / denominator * speedMultiplier);
        backLeft.setPower(  (rotY - rotX + rx) / denominator * speedMultiplier);
        frontRight.setPower((rotY - rotX - rx) / denominator * speedMultiplier);
        backRight.setPower( (rotY + rotX - rx) / denominator * speedMultiplier);
    }

    private void handleIntake() {
        if (gamepad1.square   && !lastSquare)   intake.setMode(Intake.Mode.OFF);
        if (gamepad1.cross    && !lastCross)    intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.triangle && !lastTriangle) intake.setMode(Intake.Mode.SPIT);
        if (gamepad1.circle   && !lastCircle)   intake.setMode(Intake.Mode.SHOOT);

        lastSquare   = gamepad1.square;
        lastCross    = gamepad1.cross;
        lastTriangle = gamepad1.triangle;
        lastCircle   = gamepad1.circle;
    }

    private void handleShooter() {
        if (gamepad1.dpad_down  && !lastDpadDown)  shooter.toggle();
        if (gamepad1.dpad_right && !lastDpadRight) shooter.toggleMode();
        lastDpadDown  = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad2.left_bumper  && !lastGP2L1) shooter.decreaseRPM();
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    private void handleTurret() {
        boolean l2 = gamepad1.left_trigger  > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) turret.setMode(Turret.Mode.LIMELIGHT);
        if (r2 && !lastR2) turret.setMode(Turret.Mode.ODOMETRY);

        if (!l2 && !r2) {
            turret.setMode(Turret.Mode.MANUAL);
            double manualInput = gamepad2.left_stick_x;
            if (Math.abs(manualInput) > 0.1) turret.setManualAngle(manualInput * 60.0);
        }

        lastL2 = l2;
        lastR2 = r2;
    }

    private void handleSWM() {
        if (gamepad1.options && !lastOptions) {
            swm.toggle();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;
    }

    private void handleLocalization() {
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            boolean success = limelight.megaTag2Localize(follower);
            if (success) { gamepad2.rumble(1000); gamepad1.rumble(500); }
            else          { gamepad2.rumbleBlips(3); }
        }
        lastGP2DpadUp = gamepad2.dpad_up;
    }

    private void displayTelemetry() {
        Pose pose = follower.getPose();
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset);

        telemetry.addLine("╔═══ BLUE TURRET TELE ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Drive",   FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("│ Heading", "%.1f°  (R3 to reset)", headingDeg);

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X / Y",   "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Mode",    turret.getCurrentMode());
        telemetry.addData("│ Angle",   "%.1f°", turret.getTargetAngle());
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES ✓" : "NO");

        telemetry.addLine("╠═══ LIMELIGHT ═══╣");
        telemetry.addData("│ Tag ID", limelight.getDetectedTagId());
        telemetry.addData("│ Tags",   limelight.getVisibleTagCount());
        telemetry.addData("│ TX",     "%.1f°", limelight.getTx());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled",  swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Ready",    swm.isReadyToShoot() ? "SHOOT NOW" : "Waiting");
        telemetry.addData("│ Distance", "%.1f in", swm.getDistanceForRPM());

        telemetry.addLine("╚════════════════════════╝");
        telemetry.update();
    }
}