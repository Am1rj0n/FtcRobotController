package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.tracking.AutoTracker;
import org.firstinspires.ftc.teamcode.paths.robotv2.ball12blue;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorShooterHelper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;

@TeleOp(name = "TeleOp Tracking", group = "TeleOp")
public class TeleOpTracking extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private DcMotor intakeMotor, transferMotor;
    private DistanceSensor intakeSensor;
    private Servo shooterLight, jamLight;
    private IMU imu;

    private MecanumDrive drive;
    private PinpointLocalizer pinpoint;
    private AutoTracker tracker;
    private Follower follower;

    private DualMotorShooterHelper shooter;

    private boolean shooterEnabled = false;
    private boolean jammed = false;
    private boolean objectDetected = false;

    private ElapsedTime jamTimer = new ElapsedTime();
    private ElapsedTime blinkTimer = new ElapsedTime();
    private boolean blinkState = false;

    private static final double LOW_SPEED = 0.4;
    private static final double HIGH_SPEED = 1.0;
    private static final double MOVE_DEADBAND = 0.05;
    private static final double TURN_DEADBAND = 0.05;
    private static final double JAM_DISTANCE_CM = 10.0;
    private static final double JAM_TIME = 0.2;
    private static final double LIGHT_GREEN = 0.35;
    private static final double LIGHT_RED = 0.0;
    private static final double BLINK_RATE = 0.3;

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "front_left_motor");
        fr = hardwareMap.get(DcMotor.class, "front_right_motor");
        bl = hardwareMap.get(DcMotor.class, "back_left_motor");
        br = hardwareMap.get(DcMotor.class, "back_right_motor");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        shooterLight = hardwareMap.get(Servo.class, "shooter_light");
        jamLight = hardwareMap.get(Servo.class, "jam_light");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        drive = new MecanumDrive(fl, fr, bl, br);
        pinpoint = new PinpointLocalizer(imu);
        tracker = new AutoTracker(2.2, 0.6);

        shooter = new DualMotorShooterHelper(hardwareMap, Constants.shooterCoefficients);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(ball12blue.autoEndPose);

        DcMotor[] motors = { fl, fr, bl, br, intakeMotor, transferMotor };
        for (DcMotor m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            updateJam();
            handleDrive();
            handleShooter();
            updateLights();

            shooter.update();
            follower.update();

            sleep(20);
        }

        stopAll();
    }

    private void handleDrive() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double manualTurn = gamepad1.right_stick_x;

        double speed = gamepad1.left_stick_button ? LOW_SPEED : HIGH_SPEED;
        double heading = pinpoint.getHeadingRad();

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        boolean moving = Math.hypot(x, y) > MOVE_DEADBAND;
        boolean turning = Math.abs(manualTurn) > TURN_DEADBAND;

        double turn = manualTurn;

        if (moving && !turning) {
            double targetHeading = Math.atan2(rotY, rotX);
            turn = tracker.getTurn(targetHeading, heading);
        }

        drive.drive(rotX * speed, rotY * speed, turn * speed);
    }

    private void updateJam() {
        double d = intakeSensor.getDistance(DistanceUnit.CM);
        if (d < JAM_DISTANCE_CM) {
            if (!objectDetected) {
                objectDetected = true;
                jamTimer.reset();
            } else if (jamTimer.seconds() > JAM_TIME) {
                jammed = true;
            }
        } else {
            objectDetected = false;
            jammed = false;
        }
    }

    private void handleShooter() {
        if (gamepad1.right_bumper) shooterEnabled = true;
        if (gamepad1.left_bumper) shooterEnabled = false;

        if (shooterEnabled) shooter.runCloseShot();
        else shooter.stop();
    }

    private void updateLights() {
        if (shooterEnabled) {
            if (shooter.isAtTargetVelocity()) {
                if (blinkTimer.seconds() > BLINK_RATE) {
                    blinkState = !blinkState;
                    blinkTimer.reset();
                }
                shooterLight.setPosition(blinkState ? LIGHT_GREEN : 1.0);
            } else shooterLight.setPosition(LIGHT_GREEN);
        } else shooterLight.setPosition(LIGHT_RED);

        jamLight.setPosition(jammed ? LIGHT_RED : LIGHT_GREEN);
    }

    private void stopAll() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        shooter.stop();
    }
}
