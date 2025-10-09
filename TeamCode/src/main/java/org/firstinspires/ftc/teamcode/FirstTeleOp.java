package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumTeleOp_POV", group = "TeleOp")
public class FirstTeleOp extends OpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private IMU imu;

    // Speed modifier starting at 50%
    private double speedModifier = 0.5;

    // Track previous bumper states for edge detection
    private boolean prevRightBumper = false;
    private boolean prevLeftBumper = false;

    @Override
    public void init() {
        frontleft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontright = hardwareMap.get(DcMotor.class, "front_right_motor");
        backleft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backright = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(imuParams);

        telemetry.addLine("IMU Initialized — Ready for Field-Centric Control");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Edge detection for right bumper (increase speed)
        if (gamepad1.right_bumper && !prevRightBumper) {
            speedModifier += 0.1; // Increase speed by 10%
            if (speedModifier > 1.0) speedModifier = 1.0; // Cap at 100%
        }
        prevRightBumper = gamepad1.right_bumper;

        // Edge detection for left bumper (decrease speed)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            speedModifier -= 0.1; // Decrease speed by 10%
            if (speedModifier < 0.0) speedModifier = 0.0; // Floor at 0%
        }
        prevLeftBumper = gamepad1.left_bumper;

        // Read IMU angles
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        double robotAngle = Math.toRadians(yaw);

        // Joystick inputs
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yawInput = gamepad1.right_stick_x;

        // Field-centric transform
        double temp = axial * Math.cos(robotAngle) - lateral * Math.sin(robotAngle);
        lateral = axial * Math.sin(robotAngle) + lateral * Math.cos(robotAngle);
        axial = temp;

        // Calculate motor powers
        double leftFrontPower  = axial + lateral + yawInput;
        double rightFrontPower = axial - lateral - yawInput;
        double leftBackPower   = axial - lateral + yawInput;
        double rightBackPower  = axial + lateral - yawInput;

        // Normalize motor powers
        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply speed modifier
        frontleft.setPower(leftFrontPower * speedModifier);
        frontright.setPower(rightFrontPower * speedModifier);
        backleft.setPower(leftBackPower * speedModifier);
        backright.setPower(rightBackPower * speedModifier);

        // Reset IMU heading with 'A'
        if (gamepad1.a) {
            imu.resetYaw();
        }

        // Telemetry output - including current speed modifier
        telemetry.addData("Yaw (deg)", yaw);
        telemetry.addData("Pitch (deg)", pitch);
        telemetry.addData("Roll (deg)", roll);
        telemetry.addData("Axial (forward/back)", axial);
        telemetry.addData("Lateral (strafe)", lateral);
        telemetry.addData("Yaw Input (rotation)", yawInput);
        telemetry.addData("Speed Modifier", String.format("%.0f%%", speedModifier * 100));
        telemetry.addLine("Use LB/RB to decrease/increase speed by 10%");
        telemetry.addLine("Press A to reset heading");
        telemetry.update();
    }
}
