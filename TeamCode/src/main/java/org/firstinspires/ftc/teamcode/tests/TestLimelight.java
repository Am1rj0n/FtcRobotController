package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp(name = "Test: Limelight", group = "Tests")
public class TestLimelight extends OpMode {

    private Limelight limelight;
    private Follower follower;
    private final ElapsedTime runtime = new ElapsedTime();

    private boolean lastSquare = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        limelight = new Limelight(hardwareMap, false);

        telemetry.addLine("Limelight Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Square: Force Localization Update");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        limelight.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // Manual localization update trigger
        if (gamepad1.square && !lastSquare) {
            limelight.updateLocalization(follower, runtime.seconds());
            gamepad1.rumble(200);
        }
        lastSquare = gamepad1.square;

        // Auto localization update
        limelight.updateLocalization(follower, runtime.seconds());

        boolean hasTag = limelight.detectAprilTag();

        Pose pose = follower.getPose();
        telemetry.addLine("╔═══ LIMELIGHT TEST ═══╗");
        telemetry.addData("│ Tag Detected", hasTag ? "YES" : "NO");
        telemetry.addData("│ Tag ID", limelight.getDetectedTagId());
        telemetry.addData("│ TX", "%.1f°", limelight.getTx());
        telemetry.addData("│ TY", "%.1f°", limelight.getTy());
        telemetry.addData("│ BotPose Valid", limelight.hasValidBotPose() ? "YES" : "NO");
        telemetry.addData("│ Last Update", "%.1fs ago", runtime.seconds() - limelight.getLastUpdateTime());
        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X", "%.1f", pose.getX());
        telemetry.addData("│ Y", "%.1f", pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addLine("╚═══════════════════╝");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}