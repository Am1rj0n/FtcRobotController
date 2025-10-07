package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumTeleOP_POV", group = "TeleOp")
public class FirstTeleOp extends OpMode {

    private DcMotor frontleft, frontright, backleft, backright;
    private IMU imu;

    @Override
    public void init() {
        // Initialize motors
        frontleft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontright = hardwareMap.get(DcMotor.class, "front_right_motor");
        backleft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backright = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set motor directions
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        // ✅ IMU Setup — built-in BHI260AP (adjust orientation if needed)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP, //robot control hub logo is facing up and the usb is facing the right
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(imuParams);

        telemetry.addLine("IMU Initialized — Ready for Field-Centric Control");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read IMU yaw angle (heading) in degrees
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

        // Convert yaw to radians for math
        double robotAngle = Math.toRadians(yaw);

        // Joystick inputs
        double axial = -gamepad1.left_stick_y;  // forward/back
        double lateral = gamepad1.left_stick_x; // strafe
        double yawInput = gamepad1.right_stick_x; // rotation

        // Field-centric transform
        double temp = axial * Math.cos(robotAngle) - lateral * Math.sin(robotAngle);
        lateral = axial * Math.sin(robotAngle) + lateral * Math.cos(robotAngle);
        axial = temp;

        // Mecanum drive power calculations
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

        // Set motor power
        frontleft.setPower(leftFrontPower);
        frontright.setPower(rightFrontPower);
        backleft.setPower(leftBackPower);
        backright.setPower(rightBackPower);

        // Optional: Reset heading with 'A' button
        if (gamepad1.a) {
            imu.resetYaw();
        }

        // Telemetry for debugging
        telemetry.addData("Yaw (deg)", yaw);
        telemetry.addData("Pitch (deg)", pitch);
        telemetry.addData("Roll (deg)", roll);
        telemetry.addData("Axial", axial);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("Yaw Input", yawInput);
        telemetry.addLine("Press A to reset heading");
        telemetry.update();
    }
}


//Logo facing UP
//USB port facing RIGHT