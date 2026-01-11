package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mecanum Motor Direction Test", group="Testing")
public class MecanumMotorDirectionTest extends OpMode {

    // Declare motors
    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        // Map motors (make sure these match your configuration names in the FTC Driver Station)
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Optional: set motor directions (adjust these if needed after testing)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Press A/B/X/Y to test each motor.");
        telemetry.addLine("A = Front Left | B = Front Right");
        telemetry.addLine("X = Back Left  | Y = Back Right");
        telemetry.addLine("Hold each button to spin forward.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = 0.5; // motor test speed

        // Test each motor individually
        if (gamepad1.a) {
            frontLeft.setPower(power);
            telemetry.addLine("Front Left spinning FORWARD");
        } else {
            frontLeft.setPower(0);
        }

        if (gamepad1.b) {
            frontRight.setPower(power);
            telemetry.addLine("Front Right spinning FORWARD");
        } else {
            frontRight.setPower(0);
        }

        if (gamepad1.x) {
            backLeft.setPower(power);
            telemetry.addLine("Back Left spinning FORWARD");
        } else {
            backLeft.setPower(0);
        }

        if (gamepad1.y) {
            backRight.setPower(power);
            telemetry.addLine("Back Right spinning FORWARD");
        } else {
            backRight.setPower(0);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
