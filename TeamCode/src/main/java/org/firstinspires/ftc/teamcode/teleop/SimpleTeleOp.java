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

/**
 * Simple TeleOp - Field-Centric via REV IMU, No Pedro, No Vision
 *
 * Hardware used:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor  - drive
 *   intake (DcMotor), transfer (DcMotor)                                    - intake
 *   s1, s2 (DcMotorEx)                                                      - shooter flywheels
 *   turret (Servo)                                                           - manual servo GP2
 *   imu                                                                      - field-centric heading
 *
 * Controls:
 *   GP1 Left Stick       - Drive (field-centric)
 *   GP1 Right Stick X    - Turn
 *   GP1 L1 / R1          - Speed down / up
 *   GP1 R3               - Reset field-centric heading
 *   GP1 ▢               - Intake OFF
 *   GP1 ✕               - Intake ON
 *   GP1 △               - Intake SPIT
 *   GP1 ○               - Intake SHOOT
 *   GP1 D-Pad Down       - Shooter toggle
 *   GP1 D-Pad Right      - Shooter mode cycle
 *   GP2 Left Stick X     - Turret servo
 *   GP2 Touchpad         - Recenter turret
 *   GP2 L1 / R1          - RPM down / up
 */
@TeleOp(name = "Simple TeleOp", group = "Competition")
public class SimpleTeleOp extends OpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU for field-centric heading
    private IMU imu;
    private double fieldCentricOffset = 0.0;

    // Subsystems
    private Intake intake;
    private Shooter shooter;

    // Turret servo
    private Servo turretServo;

    private final ElapsedTime runtime = new ElapsedTime();

    // Drive speed
    private double speedMultiplier = 0.7;
    private static final double MIN_SPEED       = 0.3;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    // Turret servo
    private double turretPosition    = 0.5;
    private static final double TURRET_MIN    = 0.0;
    private static final double TURRET_MAX    = 1.0;
    private static final double TURRET_CENTER = 0.5;
    private static final double TURRET_SPEED  = 0.008; // tune if too fast/slow

    // Debouncing
    private boolean lastSquare      = false;
    private boolean lastCross       = false;
    private boolean lastTriangle    = false;
    private boolean lastCircle      = false;
    private boolean lastDpadDown    = false;
    private boolean lastDpadRight   = false;
    private boolean lastL1          = false;
    private boolean lastR1          = false;
    private boolean lastR3          = false;
    private boolean lastGP2L1       = false;
    private boolean lastGP2R1       = false;
    private boolean lastGP2Touchpad = false;

    @Override
    public void init() {
        // Drive motors
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

        // IMU - adjust LogoFacingDirection/UsbFacingDirection to match your Control Hub orientation
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // Subsystems
        intake  = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Turret servo
        turretServo = hardwareMap.get(Servo.class, "turret");
        turretServo.setPosition(TURRET_CENTER);

        telemetry.addLine("╔═══ SIMPLE TELEOP READY ═══╗");
        telemetry.addLine("│ Field-Centric via IMU      │");
        telemetry.addLine("│ GP1 R3 = Reset Heading     │");
        telemetry.addLine("│ GP2 = Turret + RPM         │");
        telemetry.addLine("╚════════════════════════════╝");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        fieldCentricOffset = 0.0;
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

    // ==================== DRIVE (FIELD-CENTRIC) ====================
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        // Speed control
        if (gamepad1.left_bumper && !lastL1) {
            speedMultiplier = Math.max(MIN_SPEED, speedMultiplier - SPEED_INCREMENT);
        }
        if (gamepad1.right_bumper && !lastR1) {
            speedMultiplier = Math.min(MAX_SPEED, speedMultiplier + SPEED_INCREMENT);
        }
        lastL1 = gamepad1.left_bumper;
        lastR1 = gamepad1.right_bumper;

        // Field-centric reset (R3)
        if (gamepad1.right_stick_button && !lastR3) {
            imu.resetYaw();
            fieldCentricOffset = 0.0;
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        // Get heading from IMU
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset;

        // Rotate joystick input to field frame
        double rotX = strafe  * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe  * Math.sin(-heading) + forward * Math.cos(-heading);

        // Mecanum calculation
        double denom  = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
        double flPower = (rotY + rotX + turn) / denom;
        double blPower = (rotY - rotX + turn) / denom;
        double frPower = (rotY - rotX - turn) / denom;
        double brPower = (rotY + rotX - turn) / denom;

        frontLeft.setPower(flPower  * speedMultiplier);
        backLeft.setPower(blPower   * speedMultiplier);
        frontRight.setPower(frPower * speedMultiplier);
        backRight.setPower(brPower  * speedMultiplier);
    }

    // ==================== INTAKE ====================
    private void handleIntake() {
        if (gamepad1.square && !lastSquare)     intake.setMode(Intake.Mode.OFF);
        if (gamepad1.cross && !lastCross)       intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.triangle && !lastTriangle) intake.setMode(Intake.Mode.SPIT);
        if (gamepad1.circle && !lastCircle)     intake.setMode(Intake.Mode.SHOOT);

        lastSquare   = gamepad1.square;
        lastCross    = gamepad1.cross;
        lastTriangle = gamepad1.triangle;
        lastCircle   = gamepad1.circle;
    }

    // ==================== SHOOTER ====================
    private void handleShooter() {
        if (gamepad1.dpad_down && !lastDpadDown)   shooter.toggle();
        if (gamepad1.dpad_right && !lastDpadRight) shooter.toggleMode();
        lastDpadDown  = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad2.left_bumper && !lastGP2L1)  shooter.decreaseRPM();
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    // ==================== TURRET (GP2 MANUAL) ====================
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

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset
        );

        telemetry.addLine("╔═══ SIMPLE TELEOP ═══╗");
        telemetry.addData("│ Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Heading", "%.1f°  (GP1 R3 to reset)", headingDeg);

        telemetry.addLine("╠═══ INTAKE ═══╣");
        telemetry.addData("│ Mode", intake.getCurrentMode());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ Active",      shooter.isActive() ? "YES" : "OFF");
        telemetry.addData("│ Mode",        shooter.getModeName());
        telemetry.addData("│ Target RPM",  "%.0f", shooter.getTargetRPM());
        telemetry.addData("│ Current RPM", "%.0f", shooter.getReadRPM());
        telemetry.addData("│ At Speed",    shooter.isAtSpeed() ? "YES ✓" : "NO");

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Position", "%.2f", turretPosition);

        telemetry.addLine("╚══════════════════════╝");
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