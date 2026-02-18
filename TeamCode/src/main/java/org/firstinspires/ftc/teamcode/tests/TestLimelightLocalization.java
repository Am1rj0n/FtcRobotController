package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * Limelight Localization Test - MegaTag2 only, no fusion
 *
 * Hardware used:
 *   Drive motors (via Pedro)
 *   Pinpoint odometry (inside Pedro)
 *   limelight (Limelight3A)
 *
 * Controls:
 *   GP1 Left/Right Stick  - Drive
 *   GP1 D-Pad Up          - MegaTag2 localize (full pose override)
 *   GP1 Share             - Reset pose to center (72, 72)
 *   GP1 R3                - Reset field-centric heading
 */
@TeleOp(name = "Test: Limelight Localization", group = "Tests")
public class TestLimelightLocalization extends OpMode {

    private Follower follower;
    private Limelight limelight;
    private Drivetrain drivetrain;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean lastDpadUp = false;
    private boolean lastShare  = false;
    private boolean lastR3     = false;

    private int megaTag2Count = 0;
    private int megaTag2FailCount = 0;

    // Track last successful MegaTag2 pose for comparison
    private double lastMT2X = 0, lastMT2Y = 0;
    private double lastMT2Time = -1;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        limelight = new Limelight(hardwareMap, false); // Blue alliance
        drivetrain = new Drivetrain(hardwareMap, follower, false);

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        telemetry.addLine("╔═══ LIMELIGHT MT2 TEST ═══╗");
        telemetry.addLine("│ D-Up:  MegaTag2 Localize  │");
        telemetry.addLine("│ Share: Reset Pose         │");
        telemetry.addLine("│ R3:    Reset FC Heading   │");
        telemetry.addLine("│ Drive near tags to test   │");
        telemetry.addLine("╚═══════════════════════════╝");
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

        // CRITICAL: feed heading to Limelight every loop for MegaTag2
        limelight.updateMegaTag2Orientation(follower);

        // MegaTag2 localize on button press
        if (gamepad1.dpad_up && !lastDpadUp) {
            Pose before = follower.getPose();
            boolean success = limelight.megaTag2Localize(follower);

            if (success) {
                Pose after = follower.getPose();
                megaTag2Count++;
                lastMT2X = after.getX();
                lastMT2Y = after.getY();
                lastMT2Time = runtime.seconds();

                // Show correction delta via rumble duration
                double delta = Math.hypot(after.getX() - before.getX(), after.getY() - before.getY());
                gamepad1.rumble(delta > 5.0 ? 1000 : 400); // Long = big correction, short = small
            } else {
                megaTag2FailCount++;
                gamepad1.rumbleBlips(3);
            }
        }
        lastDpadUp = gamepad1.dpad_up;

        // Reset pose to field center
        if (gamepad1.share && !lastShare) {
            follower.setPose(new Pose(72, 72, Math.toRadians(0)));
            megaTag2Count = 0;
            megaTag2FailCount = 0;
            lastMT2Time = -1;
            gamepad1.rumbleBlips(2);
        }
        lastShare = gamepad1.share;

        // Field-centric reset
        if (gamepad1.right_stick_button && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        // Drive (half speed for testing)
        double forward = -gamepad1.left_stick_y * 0.5;
        double strafe  =  gamepad1.left_stick_x * 0.5;
        double turn    =  gamepad1.right_stick_x * 0.5;
        drivetrain.drive(forward, strafe, turn);

        displayTelemetry();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        limelight.stop();
    }

    private void displayTelemetry() {
        Pose pose = follower.getPose();
        int tagCount = limelight.getVisibleTagCount();
        boolean alignTag = limelight.isAlignmentTagVisible();

        telemetry.addLine("╔═══ LIMELIGHT MT2 TEST ═══╗");

        telemetry.addLine("╠═══ VISION ═══╣");
        telemetry.addData("│ Tags Visible",    tagCount);
        telemetry.addData("│ Tag ID",          limelight.getDetectedTagId());
        telemetry.addData("│ TX Offset",       "%.1f°", limelight.getTx());
        telemetry.addData("│ Alignment Tag",   alignTag ? "YES ✓" : "NO");

        // MegaTag2 readiness indicator
        String mt2Status;
        if (tagCount >= 2)     mt2Status = "EXCELLENT (" + tagCount + " tags)";
        else if (tagCount == 1) mt2Status = "OK (1 tag, 2+ preferred)";
        else                    mt2Status = "NO TAGS - move closer";
        telemetry.addData("│ MT2 Ready",       mt2Status);

        telemetry.addLine("╠═══ CURRENT POSE (Odometry) ═══╣");
        telemetry.addData("│ X",       "%.2f in", pose.getX());
        telemetry.addData("│ Y",       "%.2f in", pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ MEGATAG2 STATS ═══╣");
        telemetry.addData("│ Successes", megaTag2Count);
        telemetry.addData("│ Failures",  megaTag2FailCount);
        if (lastMT2Time >= 0) {
            telemetry.addData("│ Last Fix X/Y", "%.1f, %.1f", lastMT2X, lastMT2Y);
            telemetry.addData("│ Last Fix Age", "%.1fs ago", runtime.seconds() - lastMT2Time);
        } else {
            telemetry.addData("│ Last Fix", "None yet");
        }

        telemetry.addLine("╠═══ INSTRUCTIONS ═══╣");
        telemetry.addLine("│ 1. Drive near field perimeter");
        telemetry.addLine("│ 2. Watch Tags Visible go up");
        telemetry.addLine("│ 3. Press D-Up for MT2 fix");
        telemetry.addLine("│ 4. Long rumble = big correction");
        telemetry.addLine("│ 5. Short rumble = already accurate");

        telemetry.addLine("╚═══════════════════════════════╝");
        telemetry.update();
    }
}