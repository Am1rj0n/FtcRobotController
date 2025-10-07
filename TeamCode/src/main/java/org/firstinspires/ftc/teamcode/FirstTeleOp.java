package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name = "MecanumTeleOP_POV", group = "TeleOp")
public class FirstTeleOp extends OpMode {


    private DcMotor frontleft, frontright, backleft, backright;
    private BNO055IMU imu;
    private Orientation angles;


    @Override
    public void init() {
        frontleft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontright = hardwareMap.get(DcMotor.class, "front_right_motor");
        backleft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backright = hardwareMap.get(DcMotor.class, "back_right_motor");


        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);


        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.addLine("Initialized — Ready for Field-Centric Control");
        telemetry.update();
    }


    @Override
    public void loop() {
        // Read IMU heading
        angles = imu.getAngularOrientation();
        double robotAngle = angles.firstAngle;


        // Joystick inputs
        double axial = -gamepad1.left_stick_y; // forward/back
        double lateral = gamepad1.left_stick_x; // strafe
        double yaw = gamepad1.right_stick_x; // rotation


        // Field-centric transform
        double temp = axial * Math.cos(robotAngle) - lateral * Math.sin(robotAngle);
        lateral = axial * Math.sin(robotAngle) + lateral * Math.cos(robotAngle);
        axial = temp;


        // Mecanum math
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;


        // Normalize
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }


        // Set power
        frontleft.setPower(leftFrontPower);
        frontright.setPower(rightFrontPower);
        backleft.setPower(leftBackPower);
        backright.setPower(rightBackPower);


        // Telemetry
        telemetry.addData("Robot Angle (rad)", robotAngle);
        telemetry.addData("Axial", axial);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
    }
}

