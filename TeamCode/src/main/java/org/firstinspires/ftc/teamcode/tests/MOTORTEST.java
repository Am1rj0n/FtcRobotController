package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Two Motor Test", group = "Test")
public class MOTORTEST extends LinearOpMode {

    DcMotor motor1;
    DcMotor motor2;

    double speed = 0.5;                 // starting speed
    final double SPEED_STEP = 0.05;

    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");


        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // Increase speed
            if (gamepad1.right_bumper) {
                speed += SPEED_STEP;
                sleep(150); // debounce
            }

            // Decrease speed
            if (gamepad1.left_bumper) {
                speed -= SPEED_STEP;
                sleep(150);
            }

            // Clamp speed
            speed = Range.clip(speed, 0.0, 1.0);

            // Forward
            if (gamepad1.a) {
                motor1.setPower(speed);
                motor2.setPower(speed);
            }
            // Reverse
            else if (gamepad1.b) {
                motor1.setPower(-speed);
                motor2.setPower(-speed);
            }
            // Stop
            else {
                motor1.setPower(0);
                motor2.setPower(0);
            }

            telemetry.addData("Speed Setting", speed);
            telemetry.addData("Motor1 Power", motor1.getPower());
            telemetry.addData("Motor2 Power", motor2.getPower());
            telemetry.update();
        }
    }
}
