package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;


@Autonomous(name = "MecanumAUTO_POV", group = "Autonomous")
public class FirstAUTO extends LinearOpMode {


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private BNO055IMU imu;


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


        // --- Initialize IMU ---
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);


        telemetry.addLine("Initializing IMU...");
        telemetry.update();
        sleep(1000); // give it a second to initialize
        telemetry.addLine("IMU Ready!");
        telemetry.update();


        // --- Wait for Start ---
        waitForStart();


        if (opModeIsActive()) {
            runAutonomousMode();
        }
    }


    private void runAutonomousMode() {
        // Move relative to FIELD direction, not robot direction
        drivePOV(0.5, 0, 0, 2.0);   // Forward
        drivePOV(-0.5, 0, 0, 2.0);  // Backward
        drivePOV(0, 0.5, 0, 2.0);   // Strafe Right
        drivePOV(0, -0.5, 0, 2.0);  // Strafe Left
        drivePOV(0, 0, 0.5, 1.5);   // Rotate Clockwise
        drivePOV(0, 0, -0.5, 1.5);  // Rotate Counter-Clockwise
        stopAll();
    }


    // --- Field-Centric Movement (POV mode) ---
    private void drivePOV(double axial, double lateral, double yaw, double timeSeconds) {
        double heading = getHeadingRadians();


        // Field-centric transformation
        double rotatedX = lateral * Math.cos(heading) - axial * Math.sin(heading);
        double rotatedY = lateral * Math.sin(heading) + axial * Math.cos(heading);


        // Mecanum wheel power math
        double leftFrontPower  = rotatedY + rotatedX + yaw;
        double rightFrontPower = rotatedY - rotatedX - yaw;
        double leftBackPower   = rotatedY - rotatedX + yaw;
        double rightBackPower  = rotatedY + rotatedX - yaw;


        // Normalize powers
        double max = Math.max(1.0, Math.abs(leftFrontPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


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


    // --- Get Heading from IMU ---
    private double getHeadingRadians() {
        // Convert IMU’s angle (in degrees) to radians for trig functions
        return Math.toRadians(imu.getAngularOrientation().firstAngle);
    }


    // --- Safe Delay While Allowing Simulator Updates ---
    private void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            idle();
        }
    }
}
