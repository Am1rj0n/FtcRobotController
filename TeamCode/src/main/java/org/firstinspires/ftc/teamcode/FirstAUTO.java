package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "MecanumAUTO_POV", group = "Autonomous")
public class FirstAUTO extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // --- Map Motors ---
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        // --- Reverse Right Side for Mecanum ---
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // --- Set Brake Mode ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ✅ Initialize the built-in IMU (BHI260AP)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,     // ✅ adjust if needed
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT    // ✅ adjust if needed
                )
        );
        imu.initialize(imuParams);

        telemetry.addLine("IMU Initialized — Ready!");
        telemetry.update();
        sleep(1000); // Give time to finish initializing

        // --- Wait for Start ---
        waitForStart();

        if (opModeIsActive()) {
            runAutonomousMode();
        }
    }

    private void runAutonomousMode() {
        // Move relative to FIELD direction, not robot direction
        drivePOV(0.5, 0, 0, 3.8);   // Forward
        sleep(1000);

        drivePOV(-0.5, 0, 0, 2.0);  // Backward
        sleep(1000);

        drivePOV(0, 0.5, 0, 2.0);   // Strafe Right
        sleep(1000);

        drivePOV(0, -0.5, 0, 4.0);  // Strafe Left
        sleep(1000);

        drivePOV(0, 0.5, 0, 2.0);  // Strafe Right
        sleep(1000);

        drivePOV(0, 0, 0.5, 1.8);   // Rotate Clockwise
        sleep(1000);

        drivePOV(0, 0, -0.5, 1.8);  // Rotate Counter-Clockwise
        sleep(1000);
        stopAll();
    }

    // --- Field-Centric Drive ---
    private void drivePOV(double axial, double lateral, double yaw, double timeSeconds) {
        double heading = getHeadingRadians();

        // Field-centric transformation
        double rotatedX = lateral * Math.cos(heading) - axial * Math.sin(heading);
        double rotatedY = lateral * Math.sin(heading) + axial * Math.cos(heading);

        // Mecanum wheel math
        double leftFrontPower  = rotatedY + rotatedX + yaw;
        double rightFrontPower = rotatedY - rotatedX - yaw;
        double leftBackPower   = rotatedY - rotatedX + yaw;
        double rightBackPower  = rotatedY + rotatedX - yaw;

         // Normalize powers
        double max = Math.max(1.0,
                Math.max(Math.abs(leftFrontPower),
                        Math.max(Math.abs(rightFrontPower),
                                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        frontLeft.setPower(leftFrontPower / max);
        frontRight.setPower(rightFrontPower / max);
        backLeft.setPower(leftBackPower / max);
        backRight.setPower(rightBackPower / max);


        safeWaitSeconds(timeSeconds);
        stopAll();
    }

    // --- Stop All Motors ---
    private void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // --- Get Yaw from IMU in Radians ---
    private double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // --- Wait for X seconds while keeping OpMode responsive ---
    private void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            telemetry.addData("Heading (deg)", Math.toDegrees(getHeadingRadians()));
            telemetry.update();
            idle();
        }
    }
}



