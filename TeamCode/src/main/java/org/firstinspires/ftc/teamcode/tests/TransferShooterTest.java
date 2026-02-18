package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "TransferShooterVelocity", group = "Test")
public class TransferShooterTest extends LinearOpMode {

    // Transfer motors
    DcMotor motor1;
    DcMotor motor2;

    // Shooter subsystem
    Shooter shooter;

    // Transfer settings
    double transferSpeed = 1.0;              // starts at 100% power
    final double TRANSFER_SPEED_STEP = 0.05;

    @Override
    public void runOpMode() {

        // Initialize transfer motors
        motor1 = hardwareMap.get(DcMotor.class, "intake");
        motor2 = hardwareMap.get(DcMotor.class, "transfer");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize shooter
        shooter = new Shooter(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Transfer Fwd, B=Transfer Rev");
        telemetry.addData("", "X=Shooter Toggle, Y=Shooter Mode");
        telemetry.addData("", "DPad Up/Down=Shooter RPM");
        telemetry.addData("", "RB/LB=Transfer Speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========== TRANSFER MOTOR CONTROLS ==========

            // Adjust transfer speed
            if (gamepad1.right_bumper) {
                transferSpeed += TRANSFER_SPEED_STEP;
                sleep(150); // debounce
            }

            if (gamepad1.left_bumper) {
                transferSpeed -= TRANSFER_SPEED_STEP;
                sleep(150);
            }

            // Clamp transfer speed
            transferSpeed = Range.clip(transferSpeed, 0.0, 1.0);

            // Transfer forward (A button)
            if (gamepad1.a) {
                motor1.setPower(transferSpeed);
                motor2.setPower(transferSpeed);
            }
            // Transfer reverse (B button)
            else if (gamepad1.b) {
                motor1.setPower(transferSpeed);
                motor2.setPower(-transferSpeed);
            }
            // Stop transfer
            else {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            // ========== SHOOTER CONTROLS ==========

            // Toggle shooter on/off (X button)
            if (gamepad1.x) {
                shooter.toggle();
                sleep(200); // debounce
            }

            // Toggle shooter mode close/far (Y button)
            if (gamepad1.y) {
                shooter.toggleMode();
                sleep(200);
            }

            // Increase shooter RPM (DPad Up)
            if (gamepad1.dpad_up) {
                shooter.increaseRPM();
                sleep(150);
            }

            // Decrease shooter RPM (DPad Down)
            if (gamepad1.dpad_down) {
                shooter.decreaseRPM();
                sleep(150);
            }

            // Update shooter - CRITICAL: must be called every loop
            shooter.periodic();

            // ========== TELEMETRY ==========

            telemetry.addData("=== TRANSFER ===", "");
            telemetry.addData("Speed Setting", "%.0f%%", transferSpeed * 100);
            telemetry.addData("Motor1 Power", "%.2f", motor1.getPower());
            telemetry.addData("Motor2 Power", "%.2f", motor2.getPower());
            telemetry.addData("", "");
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Status", shooter.getTelemetryString());
            telemetry.addData("", "");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Transfer", "A=Fwd | B=Rev | RB/LB=Speed");
            telemetry.addData("Shooter", "X=On/Off | Y=Mode | DPad=RPM");
            telemetry.update();
        }
    }
}