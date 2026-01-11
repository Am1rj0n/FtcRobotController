package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subfilesV2.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Ultimate TeleOp BLUE", group = "TeleOp")
public class UltimateTeleOpBlue extends OpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private LightingSubsystem lighting;
    private AutoPositionSubsystem autoPosition;
    private MegatagLocalization megatag;

    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    // Alliance color
    private static final boolean IS_RED = false;

    // System states
    private enum SystemMode { OFF, INTAKE, SPIT, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    private boolean lastSquare = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    private boolean lastCircle = false;

    @Override
    public void init() {
        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Try to get pose from autonomous, otherwise use default
        Pose startPose = getStartPoseFromAuto();
        follower.setStartingPose(startPose);

        // Initialize all subsystems
        drivetrain = new DrivetrainSubsystem(hardwareMap, follower, IS_RED);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, IS_RED);
        lighting = new LightingSubsystem(hardwareMap);
        autoPosition = new AutoPositionSubsystem(follower, IS_RED);
        megatag = new MegatagLocalization(hardwareMap);

        telemetry.addLine("✓ Ultimate TeleOp BLUE Initialized");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        runtime.reset();
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        megatag.updatePeriodic(follower, runtime.seconds());

        // === GAMEPAD 1: MAIN CONTROLS ===
        handleDrive();
        handleModeSelection();
        handleAutoPositioning();

        // === GAMEPAD 2: SHOOTER ADJUSTMENTS ===
        handleShooterControls();

        // Apply current mode logic
        applySystemMode();

        // Update shooter and lighting
        shooter.update();
        updateLighting();

        // Telemetry
        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        drivetrain.stop();
    }

    // ==================== DRIVE CONTROL ====================
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Speed control with bumpers
        if (gamepad1.left_bumper) {
            drivetrain.decreaseSpeed();
        }
        if (gamepad1.right_bumper) {
            drivetrain.increaseSpeed();
        }

        // Slow mode on L3 (left stick button)
        if (gamepad1.left_stick_button) {
            drivetrain.setSlowMode(true);
        } else {
            drivetrain.setSlowMode(false);
        }

        // Field-centric reset on R3 (right stick button)
        if (gamepad1.right_stick_button) {
            drivetrain.resetFieldCentric();
        }

        // Goal tracking toggle (Options button)
        if (gamepad1.options) {
            drivetrain.toggleGoalTracking();
        }

        drivetrain.drive(forward, strafe, turn);
    }

    // ==================== MODE SELECTION ====================
    private void handleModeSelection() {
        boolean square = gamepad1.square;
        boolean cross = gamepad1.cross;
        boolean triangle = gamepad1.triangle;
        boolean circle = gamepad1.circle;

        if (square && !lastSquare) {
            currentMode = SystemMode.OFF;
        }
        if (cross && !lastCross) {
            currentMode = SystemMode.INTAKE;
        }
        if (triangle && !lastTriangle) {
            currentMode = SystemMode.SPIT;
        }
        if (circle && !lastCircle) {
            // Circle now used for canceling auto-pos, not changing mode
        }

        lastSquare = square;
        lastCross = cross;
        lastTriangle = triangle;
        lastCircle = circle;
    }

    // ==================== AUTO POSITIONING ====================
    private void handleAutoPositioning() {
        // Toggle between close/far shooting mode
        if (gamepad1.dpad_up) {
            shooter.toggleMode();
        }

        // Far shoot position
        if (gamepad1.dpad_down) {
            autoPosition.goToFarShoot();
        }

        // Close shoot position
        if (gamepad1.dpad_left) {
            autoPosition.goToCloseShoot();
        }

        // Park position
        if (gamepad1.dpad_right) {
            autoPosition.goToPark();
        }

        // Cancel auto positioning with manual input or Circle button
        if (gamepad1.circle || drivetrain.hasManualInput(gamepad1)) {
            autoPosition.cancel();
        }

        autoPosition.update();
    }

    // ==================== SHOOTER CONTROLS ====================
    private void handleShooterControls() {
        // R2 Toggle shooter on/off
        if (gamepad1.right_trigger > 0.5) {
            shooter.toggle();
        }

        // Mode selection: Circle = Close, Square = Far
        if (gamepad2.circle) {
            shooter.setCloseMode();
        }
        if (gamepad2.square) {
            shooter.setFarMode();
        }

        // Manual control mode (hold L2)
        if (gamepad2.left_trigger > 0.1) {
            shooter.setManualMode(true);

            // Adjust manual power with bumpers
            if (gamepad2.left_bumper) {
                shooter.decreaseManualPower();
            }
            if (gamepad2.right_bumper) {
                shooter.increaseManualPower();
            }
        } else {
            shooter.setManualMode(false);

            // Fine-tune learned power with D-Pad
            if (gamepad2.dpad_left) {
                shooter.decreasePower();
            }
            if (gamepad2.dpad_right) {
                shooter.increasePower();
            }
        }
    }

    // ==================== SYSTEM MODE APPLICATION ====================
    private void applySystemMode() {
        switch (currentMode) {
            case INTAKE:
                if (intake.isJammed()) {
                    intake.stop();
                } else {
                    intake.runIntake();
                }
                break;

            case SPIT:
                intake.runSpit();
                break;

            case SHOOT:
                intake.runShoot();
                // Shooter enable/disable is now controlled by R2 toggle
                break;

            case OFF:
            default:
                intake.stop();
                break;
        }
    }

    // ==================== LIGHTING ====================
    private void updateLighting() {
        // Shooter light: Blue (far), Pink (close), Blink when ready, Red when off
        if (shooter.isEnabled()) {
            if (shooter.isCloseMode()) {
                lighting.setShooterLight(shooter.isAtTarget() ?
                        LightingSubsystem.LightMode.PINK_BLINK :
                        LightingSubsystem.LightMode.PINK);
            } else {
                lighting.setShooterLight(shooter.isAtTarget() ?
                        LightingSubsystem.LightMode.BLUE_BLINK :
                        LightingSubsystem.LightMode.BLUE);
            }
        } else {
            lighting.setShooterLight(LightingSubsystem.LightMode.RED);
        }

        // Jam light: Red when jammed, Green otherwise
        lighting.setJamLight(intake.isJammed() ?
                LightingSubsystem.LightMode.RED :
                LightingSubsystem.LightMode.GREEN);

        lighting.update();
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ ULTIMATE TELEOP BLUE ═══╗");
        telemetry.addData("│ Mode", currentMode);
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Goal Track", drivetrain.isGoalTrackingEnabled() ? "ON" : "OFF");

        telemetry.addLine("╠═══ POSITION ═══╣");
        telemetry.addData("│ X", "%.1f", pose.getX());
        telemetry.addData("│ Y", "%.1f", pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ Auto-Pos", autoPosition.isActive() ? "ACTIVE" : "Manual");

        telemetry.addLine("╠═══ INTAKE ═══╣");
        telemetry.addData("│ Jammed", intake.isJammed() ? "YES" : "NO");
        telemetry.addData("│ Distance", "%.1f cm", intake.getDistance());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ Status", shooter.isEnabled() ?
                (shooter.isManualMode() ? "MANUAL" : shooter.getModeName()) : "OFF");
        telemetry.addData("│ Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("│ Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("│ At Target", shooter.isAtTarget() ? "YES" : "NO");
        telemetry.addData("│ Power", "%.3f", shooter.getCurrentPower());

        telemetry.addLine("╠═══ MEGATAG ═══╣");
        telemetry.addData("│ Enabled", megatag.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Last Update", "%.1fs ago", runtime.seconds() - megatag.getLastUpdateTime());

        telemetry.addLine("╚════════════════════════╝");
        telemetry.update();
    }

    // ==================== HELPER METHODS ====================
    private Pose getStartPoseFromAuto() {
        // Try to get pose from ball12blue
        try {
            Pose bluePose = org.firstinspires.ftc.teamcode.paths.robotv2.ball12blue.autoEndPose;
            if (bluePose != null && (bluePose.getX() != 0 || bluePose.getY() != 0)) {
                return bluePose;
            }
        } catch (Exception e) {
            telemetry.addLine("No auto end pose found, using default");
        }

        // Default blue start position
        return new Pose(48.9, 66.6, Math.toRadians(0));
    }
}