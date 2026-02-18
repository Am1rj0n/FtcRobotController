package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class RedTeleOp extends OpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;
    private ShootingWhileMoving swm;

    private Follower follower;
    private final ElapsedTime runtime = new ElapsedTime();

    private static final boolean IS_RED = true; // Only difference from Blue

    private boolean lastSquare = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    private boolean lastCircle = false;
    private boolean lastL2 = false;
    private boolean lastR2 = false;
    private boolean lastR3 = false;
    private boolean lastTouchpad = false;
    private boolean lastOptions = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastGP2L1 = false;
    private boolean lastGP2R1 = false;
    private boolean lastGP2DpadUp = false; // MegaTag localization

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        limelight = new Limelight(hardwareMap, IS_RED);
        drivetrain = new Drivetrain(hardwareMap, follower, IS_RED);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, limelight, IS_RED);
        swm = new ShootingWhileMoving(follower, shooter, turret, IS_RED);

        telemetry.addLine("=== INIT ===");
        telemetry.addLine("Waiting for Start...");
        telemetry.addData("Auto Pose Available", AutoToTeleTransfer.finalPose != null);
        telemetry.update();
    }

    @Override
    public void start() {
        // USE AUTO'S FINAL POSITION
        if (AutoToTeleTransfer.finalPose != null) {
            follower.setStartingPose(AutoToTeleTransfer.finalPose);
            gamepad1.rumble(500); // Confirm auto pose loaded
        } else {
            // Default starting position if auto wasn't run
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        }

        follower.startTeleopDrive();
        limelight.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();

        // CRITICAL: Update robot orientation for MegaTag2 every loop
        limelight.updateMegaTag2Orientation(follower);


        handleDrive();
        handleIntake();
        handleShooter();
        handleTurret();
        handleSWM();
        handleLocalization(); // MegaTag and corner reset

        if (shooter.isActive()) {
            double distanceMeters = swm.getDistanceForRPM() * 0.0254;
            shooter.setRPMForDistance(distanceMeters);
        }

        shooter.periodic();
        turret.update(follower.getPose());
        swm.update(); // Check zones and update heading lock

        // Auto-shoot when in zone
        if (swm.shouldAutoShoot()) {
            intake.setMode(Intake.Mode.SHOOT);
        }

        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        drivetrain.stop();
        limelight.stop();
    }

    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            if (drivetrain.isHolding()) {
                gamepad1.rumble(500);
            }
        }
        lastTouchpad = gamepad1.touchpad;

        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        if (gamepad1.left_bumper) drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        if (gamepad1.right_stick_button && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        drivetrain.drive(forward, strafe, turn);
    }

    private void handleIntake() {
        if (gamepad1.square && !lastSquare) {
            intake.setMode(Intake.Mode.OFF);
        }
        if (gamepad1.cross && !lastCross) {
            intake.setMode(Intake.Mode.INTAKE);
        }
        if (gamepad1.triangle && !lastTriangle) {
            intake.setMode(Intake.Mode.SPIT);
        }
        if (gamepad1.circle && !lastCircle) {
            intake.setMode(Intake.Mode.SHOOT);
        }

        lastSquare = gamepad1.square;
        lastCross = gamepad1.cross;
        lastTriangle = gamepad1.triangle;
        lastCircle = gamepad1.circle;
    }

    private void handleShooter() {
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooter.toggle();
        }
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !lastDpadRight) {
            shooter.toggleMode();
        }
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad2.left_bumper && !lastGP2L1) {
            shooter.decreaseRPM();
        }
        if (gamepad2.right_bumper && !lastGP2R1) {
            shooter.increaseRPM();
        }

        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    private void handleTurret() {
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

            double manualInput = gamepad2.left_stick_x;
            if (Math.abs(manualInput) > 0.1) {
                turret.setManualAngle(manualInput * 58.0);
            }
        }

        lastL2 = l2;
        lastR2 = r2;
    }

    private void handleSWM() {
        if (gamepad1.options && !lastOptions) {
            swm.toggle();
            if (swm.isEnabled()) {
                gamepad1.rumble(500);
            } else {
                gamepad1.rumbleBlips(2);
            }
        }
        lastOptions = gamepad1.options;
    }

    private void handleLocalization() {
        // MEGATAG2 LOCALIZATION - Gamepad 2 D-Pad Up
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            boolean success = limelight.megaTag2Localize(follower);

            if (success) {
                gamepad2.rumble(1000); // Long rumble = success
                gamepad1.rumble(500);  // Notify driver too
            } else {
                gamepad2.rumbleBlips(3); // Triple blip = failed
            }
        }
        lastGP2DpadUp = gamepad2.dpad_up;
    }

    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ RED TELEOP ═══╗");
        telemetry.addData("│ Intake", intake.getCurrentMode());
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);

        telemetry.addLine("╠═══ LOCALIZATION ═══╣");
        telemetry.addData("│ X / Y", "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Mode", turret.getCurrentMode());
        telemetry.addData("│ Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES" : "NO");

        telemetry.addLine("╠═══ LIMELIGHT ═══╣");
        telemetry.addData("│ Tag ID", limelight.getDetectedTagId());
        telemetry.addData("│ Visible Tags", limelight.getVisibleTagCount());
        telemetry.addData("│ TX", "%.1f°", limelight.getTx());
        telemetry.addData("│ MegaTag2", "GP2 D-Up");

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled", swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ In Zone", swm.isInShootZone() ? "YES" : "NO");
        telemetry.addData("│ Heading Lock", swm.isHeadingLockActive() ? "ACTIVE" : "Off");
        telemetry.addData("│ Auto-Shoot", swm.shouldAutoShoot() ? "READY" : "Waiting");
        telemetry.addData("│ Velocity", "%.1f in/s", swm.getVelocityMagnitude());
        telemetry.addData("│ Distance", "%.1f in", swm.getDistanceForRPM());

        telemetry.addLine("╚════════════════════╝");
        telemetry.update();
    }
}