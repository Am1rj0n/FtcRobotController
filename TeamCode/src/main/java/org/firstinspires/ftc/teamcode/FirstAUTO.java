package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "MecanumAUTO", group = "Autonomous")
public class FirstAUTO extends LinearOpMode {


    private DcMotor frontLeft, frontRight, backLeft, backRight;


    @Override
    public void runOpMode() {


        // Map motors
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");


        // Reverse right side
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        // Brake when power = 0
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for play button
        waitForStart();


        if (opModeIsActive() && !isStopRequested()) {
            runAutonomousMode();
        }
    }


    public void runAutonomousMode() {
        //  MOVEMENTS
        // Forward
        moveAll(0.5, 0.5, 0.5, 0.5, 1000);
        sleep(500);


        // Backward
        moveAll(-0.5, -0.5, -0.5, -0.5, 1000);
        sleep(500);


        // Spin Right
        moveAll(0.5, -0.5, 0.5, -0.5, 1000);
        sleep(500);


        // Spin Left
        moveAll(-0.5, 0.5, -0.5, 0.5, 1000);
        sleep(500);


        // Strafe Right
        moveAll(0.5, -0.5, -0.5, 0.5, 1000);
        sleep(500);


        // Strafe Left
        moveAll(-0.5, 0.5, 0.5, -0.5, 1000);
        sleep(500);


        stopAll();
    }


    // Move all motors at custom power for given time
    private void moveAll(double fl, double fr, double bl, double br, long timeMs) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
        sleep(timeMs);
        stopAll();
    }


    // Stop all motors
    private void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    // Optional safe wait method
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!isStopRequested() && timer.seconds() < time) {
            idle();
        }
    }
}


