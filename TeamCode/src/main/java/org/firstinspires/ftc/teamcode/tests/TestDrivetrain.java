package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Test: Drivetrain", group = "Tests")
public class TestDrivetrain extends OpMode {

    private Drivetrain drivetrain;
    private Follower follower;

    private boolean lastR3 = false;
    private boolean lastTouchpad = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        drivetrain = new Drivetrain(hardwareMap, follower, false);

        telemetry.addLine("Drivetrain Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick: Drive");
        telemetry.addLine("  Right Stick: Turn");
        telemetry.addLine("  L1/R1: Speed");
        telemetry.addLine("  R3: Field-Centric Reset");
        telemetry.addLine("  Touchpad: Position Hold");
        telemetry.addLine("  Share: Corner Reset");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Speed control
        if (gamepad1.left_bumper) drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        // Field-centric reset
        boolean r3 = gamepad1.right_stick_button;
        if (r3 && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = r3;

        // Position hold
        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            gamepad1.rumble(500);
        }
        lastTouchpad = gamepad1.touchpad;

        // Corner reset
        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        drivetrain.drive(forward, strafe, turn);

        Pose pose = follower.getPose();
        telemetry.addLine("╔═══ DRIVETRAIN TEST ═══╗");
        telemetry.addData("│ X", "%.1f", pose.getX());
        telemetry.addData("│ Y", "%.1f", pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Holding", drivetrain.isHolding() ? "YES" : "NO");
        telemetry.addLine("╚══════════════════════╝");
        telemetry.update();
    }
}