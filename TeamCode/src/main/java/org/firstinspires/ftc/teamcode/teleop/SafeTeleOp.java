package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "SAFE MODE", group = "Safe")
public class SafeTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private Intake   intake;
    private Shooter  shooter;
    private Servo    turretServo;

    private final ElapsedTime runtime = new ElapsedTime();

    // Set to true to enable field-centric via IMU
    private static final boolean FIELD_CENTRIC = false;
    private double fieldCentricOffset = 0.0;

    private double speedMultiplier   = 0.7;
    private static final double MIN_SPEED       = 0.3;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    private double turretPosition    = 0.5;
    private static final double TURRET_MIN    = 0.0;
    private static final double TURRET_MAX    = 1.0;
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_SPEED  = 0.01;

    private boolean lastL1          = false;
    private boolean lastR1          = false;
    private boolean lastR3          = false;
    private boolean lastDpadDown    = false;
    private boolean lastDpadRight   = false;
    private boolean lastGP2L1       = false;
    private boolean lastGP2R1       = false;
    private boolean lastGP2Touchpad = false;

    @Override
    public void init() {
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
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();

        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        turretServo = hardwareMap.get(Servo.class, "turret");
        turretServo.setPosition(TURRET_CENTER);

        telemetry.addData("Drive", FIELD_CENTRIC ? "FIELD-CENTRIC (IMU)" : "ROBOT-CENTRIC");
        telemetry.addLine("SAFE MODE - No Pinpoint / No Limelight");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        runtime.reset();
    }

    @Override
    public void loop() {
        handleDrive();
        handleIntake();
        handleShooter();
        handleTurret();
        shooter.periodic();
        displayTelemetry();
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

        double rotX, rotY;

        if (!FIELD_CENTRIC) {
            // ── ROBOT CENTRIC (active) ──────────────────────────────────────
            rotX = x;
            rotY = y;
        } else {
            // ── FIELD CENTRIC (set FIELD_CENTRIC = true to use) ────────────
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset;
            rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        }

        rotX = rotX * 1.1; // counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower( (rotY + rotX + rx) / denominator * speedMultiplier);
        backLeft.setPower(  (rotY - rotX + rx) / denominator * speedMultiplier);
        frontRight.setPower((rotY - rotX - rx) / denominator * speedMultiplier);
        backRight.setPower( (rotY + rotX - rx) / denominator * speedMultiplier);
    }

    private void handleIntake() {
        if (gamepad1.square) intake.setMode(Intake.Mode.OFF);
        if (gamepad1.cross) intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.triangle) intake.setMode(Intake.Mode.SPIT);
        if (gamepad1.circle) intake.setMode(Intake.Mode.SHOOT);
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
        double input = gamepad2.left_stick_x;
        if (Math.abs(input) > 0.05) {
            turretPosition += input * TURRET_SPEED;
            turretPosition  = Math.max(TURRET_MIN, Math.min(TURRET_MAX, turretPosition));
            turretServo.setPosition(turretPosition);
        }

        if (gamepad2.touchpad && !lastGP2Touchpad) {
            turretPosition = TURRET_CENTER;
            turretServo.setPosition(TURRET_CENTER);
            gamepad2.rumble(300);
        }
        lastGP2Touchpad = gamepad2.touchpad;
    }

    private void displayTelemetry() {
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset
        );

        telemetry.addLine("╔═══ SAFE MODE ═══╗");
        telemetry.addData("│ Drive",   FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Heading", "%.1f°  (R3 to reset)", headingDeg);

        telemetry.addLine("╠═══ INTAKE ═══╣");
        telemetry.addData("│ Mode", intake.getCurrentMode());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Position", "%.2f", turretPosition);

        telemetry.addLine("╠═══ STATUS ═══╣");
        telemetry.addLine("│ ⚠ NO Pinpoint");
        telemetry.addLine("│ ⚠ NO Limelight");
        telemetry.addLine("│ ✓ IMU active");

        telemetry.addLine("╚══════════════════╝");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}