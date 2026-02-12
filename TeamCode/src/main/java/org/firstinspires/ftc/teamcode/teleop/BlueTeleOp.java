package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class BlueTeleOp extends OpMode {

    // Subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;
    private ShootingWhileMoving swm;

    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    private static final boolean IS_RED = false;

    // Debouncing
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

    // Gamepad 2 debouncing
    private boolean lastGP2L1 = false;
    private boolean lastGP2R1 = false;

    @Override
    public void init() {
        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);


        // Initialize subsystems
        limelight = new Limelight(hardwareMap, IS_RED);
        drivetrain = new Drivetrain(hardwareMap, follower, IS_RED);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, limelight, IS_RED);
        swm = new ShootingWhileMoving(follower, shooter, turret, IS_RED);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        limelight.start();
        runtime.reset();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Update limelight localization
        limelight.updateLocalization(follower, runtime.seconds());

        // Handle inputs
        handleDrive();
        handleIntakeModes();
        handleShooterControls();
        handleTurretControls();
        handleSWM();

        // Update subsystems
        shooter.periodic(); // CRITICAL - must call every loop
        turret.update(follower.getPose());
        swm.update();


        // Display telemetry
        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        drivetrain.stop();
        limelight.stop();
    }

    // ==================== DRIVE CONTROL ====================
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Position hold toggle (Touchpad)
        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            if (drivetrain.isHolding()) {
                gamepad1.rumble(500);
            }
        }
        lastTouchpad = gamepad1.touchpad;

        // Corner reset (Share)
        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        // Speed control (L1/R1)
        if (gamepad1.left_bumper) drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        // Field-centric reset (R3)
        boolean r3 = gamepad1.right_stick_button;
        if (r3 && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = r3;

        // Apply heading lock if SWM active and in zone
        if (swm.isHeadingLockActive()) {
            drivetrain.setHeadingLock(swm.getTargetHeading());
        } else {
            drivetrain.releaseHeadingLock();
        }

        drivetrain.drive(forward, strafe, turn);
    }

    // ==================== INTAKE MODES ====================
    private void handleIntakeModes() {
        boolean square = gamepad1.square;
        boolean cross = gamepad1.cross;
        boolean triangle = gamepad1.triangle;
        boolean circle = gamepad1.circle;

        if (square && !lastSquare) {
            intake.setMode(Intake.Mode.OFF);
        }
        if (cross && !lastCross) {
            intake.setMode(Intake.Mode.INTAKE);
        }
        if (triangle && !lastTriangle) {
            intake.setMode(Intake.Mode.SPIT);
        }
        if (circle && !lastCircle) {
            // Auto-shoot if conditions met, otherwise manual shoot
            if (swm.shouldAutoShoot()) {
                intake.setMode(Intake.Mode.SHOOT);
            } else {
                intake.setMode(Intake.Mode.SHOOT);
            }
        }

        lastSquare = square;
        lastCross = cross;
        lastTriangle = triangle;
        lastCircle = circle;
    }

    // ==================== SHOOTER CONTROLS ====================
    private void handleShooterControls() {
        // D-Pad Down: Toggle shooter ON/OFF
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !lastDpadDown) {
            shooter.toggle();
        }
        lastDpadDown = dpadDown;

        // D-Pad Right: Toggle Close/Far mode
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !lastDpadRight) {
            shooter.toggleMode();
        }
        lastDpadRight = dpadRight;

        // Gamepad 2: Fine RPM adjustment
        boolean gp2L1 = gamepad2.left_bumper;
        boolean gp2R1 = gamepad2.right_bumper;

        if (gp2L1 && !lastGP2L1) {
            shooter.decreaseRPM();
        }
        if (gp2R1 && !lastGP2R1) {
            shooter.increaseRPM();
        }

        lastGP2L1 = gp2L1;
        lastGP2R1 = gp2R1;
    }

    // ==================== TURRET CONTROLS ====================
    private void handleTurretControls() {
        boolean l2 = gamepad1.left_trigger > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) {
            turret.setMode(Turret.Mode.LIMELIGHT);
        }
        if (r2 && !lastR2) {
            turret.setMode(Turret.Mode.ODOMETRY);
        }

        // If neither trigger pressed, allow manual control
        if (!l2 && !r2) {
            turret.setMode(Turret.Mode.MANUAL);

            // Gamepad 2 manual override
            double manualInput = gamepad2.left_stick_x;
            if (Math.abs(manualInput) > 0.1) {
                turret.setManualAngle(manualInput * 58.0); // Scale to ±58°
            }
        }

        lastL2 = l2;
        lastR2 = r2;
    }

    // ==================== SHOOTING WHILE MOVING ====================
    private void handleSWM() {
        boolean options = gamepad1.options;

        if (options && !lastOptions) {
            swm.toggle();
            if (swm.isEnabled()) {
                gamepad1.rumble(500);
            } else {
                gamepad1.rumbleBlips(2);
            }
        }
        lastOptions = options;

        // Auto-shoot trigger
        if (swm.shouldAutoShoot()) {
            intake.setMode(Intake.Mode.SHOOT);
        }
    }


    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ BLUE TELEOP ═══╗");
        telemetry.addData("│ Intake", intake.getCurrentMode());
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);

        telemetry.addLine("╠═══ POSITION ═══╣");
        telemetry.addData("│ X / Y", "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ Hold", drivetrain.isHolding() ? "LOCKED" : "Free");
        telemetry.addData("│ H-Lock", drivetrain.isHeadingLocked() ? "ACTIVE" : "Off");

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Mode", turret.getCurrentMode());
        telemetry.addData("│ Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("│ Aligned", turret.isAligned() ? "YES" : "NO");
        telemetry.addData("│ Distance", "%.1f in", turret.distanceToGoal(pose));

        telemetry.addLine("╠═══ LIMELIGHT ═══╣");
        telemetry.addData("│ Tag ID", limelight.getDetectedTagId());
        telemetry.addData("│ TX", "%.1f°", limelight.getTx());
        telemetry.addData("│ Last Update", "%.1fs ago", runtime.seconds() - limelight.getLastUpdateTime());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled", swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ In Zone", swm.isInShootZone() ? "YES" : "NO");
        telemetry.addData("│ Auto-Shoot", swm.shouldAutoShoot() ? "READY" : "Waiting");

        telemetry.addLine("╚════════════════════╝");
        telemetry.update();
    }

}