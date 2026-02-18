package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "MegaTag2 Backup Blue", group = "BackUp")
public class MegaTag2TeleOp extends OpMode {

    private static final boolean IS_RED = false;

    private Follower follower;
    private Limelight limelight;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private ShootingWhileMoving swm;

    private final ElapsedTime runtime = new ElapsedTime();

    private static final Pose BLUE_CORNER = new Pose(9, 9, Math.toRadians(0));
    private static final Pose RED_CORNER  = new Pose(135, 9, Math.toRadians(180));

    // Debouncing
    private boolean lastSquare       = false;
    private boolean lastCross        = false;
    private boolean lastTriangle     = false;
    private boolean lastCircle       = false;
    private boolean lastDpadDown     = false;
    private boolean lastDpadRight    = false;
    private boolean lastL1           = false;
    private boolean lastR1           = false;
    private boolean lastL2           = false;
    private boolean lastR2           = false;
    private boolean lastR3           = false; // field-centric reset
    private boolean lastOptions      = false;
    private boolean lastGP2L1        = false;
    private boolean lastGP2R1        = false;
    private boolean lastGP2DpadUp    = false;
    private boolean lastGP2Share     = false;

    private int megaTag2Count   = 0;
    private int cornerResetCount = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        limelight  = new Limelight(hardwareMap, IS_RED);
        drivetrain = new Drivetrain(hardwareMap, follower, IS_RED);
        intake     = new Intake(hardwareMap);
        shooter    = new Shooter(hardwareMap);
        turret     = new Turret(hardwareMap, limelight, IS_RED);
        swm        = new ShootingWhileMoving(follower, shooter, turret, IS_RED);

        telemetry.addLine("╔═══ MEGATAG2 BLUE INIT ═══╗");
        telemetry.addLine("│ GP1 R3: Field-Centric Reset │");
        telemetry.addLine("│ GP2 D-Up: MegaTag2          │");
        telemetry.addLine("│ GP2 Share: Corner Reset      │");
        telemetry.addData("│ Auto Pose", AutoToTeleTransfer.finalPose != null ? "YES" : "NO");
        telemetry.addLine("╚═════════════════════════════╝");
        telemetry.update();
    }

    @Override
    public void start() {
        if (AutoToTeleTransfer.finalPose != null) {
            follower.setStartingPose(AutoToTeleTransfer.finalPose);
            gamepad1.rumble(500);
        } else {
            follower.setStartingPose(IS_RED ? RED_CORNER : BLUE_CORNER);
        }

        follower.startTeleopDrive();
        limelight.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // CRITICAL: feed heading to Limelight every loop for MegaTag2
        limelight.updateMegaTag2Orientation(follower);

        handleDrive();
        handleIntake();
        handleShooter();
        handleTurret();
        handleSWM();
        handleLocalization();

        if (shooter.isActive()) {
            double distanceMeters = swm.getDistanceForRPM() * 0.0254;
            shooter.setRPMForDistance(distanceMeters);
        }

        shooter.periodic();
        turret.update(follower.getPose());
        swm.update();

        if (swm.shouldAutoShoot()) {
            intake.setMode(Intake.Mode.SHOOT);
        }

        displayTelemetry();
    }

    // ==================== DRIVE ====================
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        if (gamepad1.left_bumper && !lastL1)  drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper && !lastR1) drivetrain.increaseSpeed();
        lastL1 = gamepad1.left_bumper;
        lastR1 = gamepad1.right_bumper;

        // Field-centric reset (R3) - uses Pedro heading
        if (gamepad1.right_stick_button && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        drivetrain.drive(forward, strafe, turn);
    }

    // ==================== INTAKE ====================
    private void handleIntake() {
        if (gamepad1.square && !lastSquare)     intake.setMode(Intake.Mode.OFF);
        if (gamepad1.cross && !lastCross)       intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.triangle && !lastTriangle) intake.setMode(Intake.Mode.SPIT);
        if (gamepad1.circle && !lastCircle)     intake.setMode(Intake.Mode.SHOOT);

        lastSquare   = gamepad1.square;
        lastCross    = gamepad1.cross;
        lastTriangle = gamepad1.triangle;
        lastCircle   = gamepad1.circle;
    }

    // ==================== SHOOTER ====================
    private void handleShooter() {
        if (gamepad1.dpad_down && !lastDpadDown)   shooter.toggle();
        if (gamepad1.dpad_right && !lastDpadRight) shooter.toggleMode();
        lastDpadDown  = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad2.left_bumper && !lastGP2L1)  shooter.decreaseRPM();
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    // ==================== TURRET ====================
    private void handleTurret() {
        boolean l2 = gamepad1.left_trigger > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) turret.setMode(Turret.Mode.LIMELIGHT);
        if (r2 && !lastR2) turret.setMode(Turret.Mode.ODOMETRY);
        lastL2 = l2;
        lastR2 = r2;
    }

    // ==================== SWM ====================
    private void handleSWM() {
        if (gamepad1.options && !lastOptions) {
            swm.toggle();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;
    }

    // ==================== LOCALIZATION ====================
    private void handleLocalization() {
        // MegaTag2 - GP2 D-Pad Up
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            boolean success = limelight.megaTag2Localize(follower);
            if (success) {
                megaTag2Count++;
                gamepad2.rumble(1000);
                gamepad1.rumble(500);
            } else {
                gamepad2.rumbleBlips(3);
            }
        }
        lastGP2DpadUp = gamepad2.dpad_up;

        // Corner reset - GP2 Share
        if (gamepad2.share && !lastGP2Share) {
            Pose cornerPose = IS_RED ? RED_CORNER : BLUE_CORNER;
            follower.setPose(cornerPose);
            cornerResetCount++;
            gamepad2.rumbleBlips(2);
            gamepad1.rumbleBlips(2);
        }
        lastGP2Share = gamepad2.share;
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ MEGATAG2 BLUE ═══╗");
        telemetry.addData("│ Speed",    "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Intake",   intake.getCurrentMode());

        telemetry.addLine("╠═══ LOCALIZATION ═══╣");
        telemetry.addData("│ X / Y",         "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading",        "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ MegaTag2 Uses",  megaTag2Count);
        telemetry.addData("│ Corner Resets",  cornerResetCount);
        telemetry.addData("│ Visible Tags",   limelight.getVisibleTagCount());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ Active",     shooter.isActive() ? "YES" : "OFF");
        telemetry.addData("│ Mode",       shooter.getModeName());
        telemetry.addData("│ Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("│ Current RPM","%.0f", shooter.getReadRPM());
        telemetry.addData("│ At Speed",   shooter.isAtSpeed() ? "YES ✓" : "NO");

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Mode",    turret.getCurrentMode());
        telemetry.addData("│ Angle",   "%.1f°", turret.getTargetAngle());
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES ✓" : "NO");

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled",      swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ In Zone",      swm.isInShootZone() ? "YES" : "NO");
        telemetry.addData("│ Auto-Shoot",   swm.shouldAutoShoot() ? "READY" : "Waiting");
        telemetry.addData("│ Distance",     "%.1f in", swm.getDistanceForRPM());

        telemetry.addLine("╠═══ BUTTONS ═══╣");
        telemetry.addLine("│ GP1 R3: Field-Centric Reset");
        telemetry.addLine("│ GP2 D-Up: MegaTag2");
        telemetry.addLine("│ GP2 Share: Corner Reset");

        telemetry.addLine("╚═════════════════════╝");
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        drivetrain.stop();
        limelight.stop();
    }
}