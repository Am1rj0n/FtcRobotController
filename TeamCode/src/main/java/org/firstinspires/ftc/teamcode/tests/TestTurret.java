package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Test: Turret", group = "Tests")
public class TestTurret extends OpMode {

    private Turret turret;
    private Limelight limelight;
    private Follower follower;

    private boolean lastL2 = false;
    private boolean lastR2 = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        limelight = new Limelight(hardwareMap, false);
        turret = new Turret(hardwareMap, limelight, false);

        telemetry.addLine("Turret Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  L2: Limelight Mode");
        telemetry.addLine("  R2: Odometry Mode");
        telemetry.addLine("  Left Stick X: Manual");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();

        boolean l2 = gamepad1.left_trigger > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) {
            turret.setMode(Turret.Mode.LIMELIGHT);
        }
        if (r2 && !lastR2) {
            turret.setMode(Turret.Mode.ODOMETRY);
        }

        if (!l2 && !r2) {
            turret.setMode(Turret.Mode.MANUAL);
            double manualInput = gamepad1.left_stick_x;
            turret.setManualAngle(manualInput * 58.0);
        }

        lastL2 = l2;
        lastR2 = r2;

        turret.update(follower.getPose());

        Pose pose = follower.getPose();
        telemetry.addLine("╔═══ TURRET TEST ═══╗");
        telemetry.addData("│ Mode", turret.getCurrentMode());
        telemetry.addData("│ Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES" : "NO");
        telemetry.addData("│ Distance", "%.1f in", turret.distanceToGoalMeters(pose));
        telemetry.addData("│ Tag ID", limelight.getDetectedTagId());
        telemetry.addData("│ TX", "%.1f°", limelight.getTx());
        telemetry.addLine("╚═══════════════════╝");
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}