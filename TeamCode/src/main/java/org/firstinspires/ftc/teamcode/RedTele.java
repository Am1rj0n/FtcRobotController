package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.paths.robotv2.ball12blueClose;
import org.firstinspires.ftc.teamcode.subfilesV2.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Ultimate TeleOp RED", group = "TeleOp")
public class RedTele extends OpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private LightingSubsystem lighting;
    private AutoPositionSubsystem autoPosition;
    private LimelightHelper limelight;
    private ArtifactFetcher artifactFetcher;

    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    // Alliance
    private static final boolean IS_RED = true;

    // System states
    private enum SystemMode { OFF, INTAKE, SPIT, SHOOT }
    private SystemMode currentMode = SystemMode.OFF;

    // Debouncing
    private boolean lastSquare = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    private boolean lastR2 = false;
    private boolean lastDpadUp = false;
    private boolean lastLeftTrigger = false;
    private boolean lastTouchpad = false;
    private boolean lastCircle = false;
    private boolean lastR3 = false;

    // Time-based debouncing for shooter adjustments
    private ElapsedTime gp2AdjustTimer = new ElapsedTime();
    private static final double ADJUST_DELAY = 0.15;

    @Override
    public void init() {
        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Try to get pose from autonomous
        Pose startPose = getStartPoseFromAuto();
        follower.setStartingPose(startPose);

        // Initialize all subsystems
        drivetrain = new DrivetrainSubsystem(hardwareMap, follower, IS_RED);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, IS_RED);
        lighting = new LightingSubsystem(hardwareMap);
        autoPosition = new AutoPositionSubsystem(follower, IS_RED);
        limelight = new LimelightHelper(hardwareMap, IS_RED);
        artifactFetcher = new ArtifactFetcher(hardwareMap, follower);

        telemetry.addLine("✓ Ultimate TeleOp RED Initialized");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        limelight.start();
        runtime.reset();
        gp2AdjustTimer.reset();
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        limelight.updateLocalization(follower, runtime.seconds());

        // === GAMEPAD 1: MAIN CONTROLS ===
        handleDrive();
        handleModeSelection();
        handleAutoPositioning();
        handleShooterToggle();
        handleArtifactFetcher();

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
        limelight.stop();
    }

    // ==================== DRIVE CONTROL ====================
    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // --- TOUCHPAD: POSITION HOLD (TOGGLE) ---
        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
        }
        lastTouchpad = gamepad1.touchpad;

        // --- SHARE/BACK: CORNER RESET ---
        if (gamepad1.share || gamepad1.back) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(3);
        }

        // Speed control with bumpers
        if (gamepad1.left_bumper) drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        // Field-centric reset on R3 (right stick button) with rumble
        boolean r3 = gamepad1.right_stick_button;
        if (r3 && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = r3;

        // Goal tracking toggle (Options button)
        if (gamepad1.options) drivetrain.toggleGoalTracking();

        // The drivetrain.drive logic now handles the "Hold" check internally
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
            currentMode = SystemMode.SHOOT;
        }

        lastSquare = square;
        lastCross = cross;
        lastTriangle = triangle;
        lastCircle = circle;
    }

    // ==================== AUTO POSITIONING ====================
    private void handleAutoPositioning() {
        // Toggle between close/far shooting mode
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) {
            shooter.toggleMode();
        }
        lastDpadUp = dpadUp;

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

        // Cancel auto positioning with manual input or L3
        if (gamepad1.left_stick_button || drivetrain.hasManualInput(gamepad1)) {
            autoPosition.cancel();
        }

        autoPosition.update();
    }

    // ==================== SHOOTER TOGGLE ====================
    private void handleShooterToggle() {
        boolean r2 = gamepad1.right_trigger > 0.5;
        if (r2 && !lastR2) {
            shooter.toggle();
        }
        lastR2 = r2;
    }

    // ==================== ARTIFACT FETCHER ====================
    private void handleArtifactFetcher() {
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        if (leftTrigger && !lastLeftTrigger) {
            artifactFetcher.toggle();
        }
        lastLeftTrigger = leftTrigger;

        // Update artifact fetcher
        if (artifactFetcher.isActive()) {
            artifactFetcher.update();

            // Auto-enable intake only if not jammed
            if (currentMode == SystemMode.OFF && !intake.isJammed()) {
                currentMode = SystemMode.INTAKE;
            }
        }
    }

    // ==================== SHOOTER CONTROLS ====================
    private void handleShooterControls() {
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

            // D-Pad adjusts manual powers by ±5% (time-debounced)
            if (gp2AdjustTimer.seconds() > ADJUST_DELAY) {
                if (gamepad2.dpad_up) {
                    shooter.increaseManualBottomPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_down) {
                    shooter.decreaseManualBottomPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_right) {
                    shooter.increaseManualTopPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_left) {
                    shooter.decreaseManualTopPower();
                    gp2AdjustTimer.reset();
                }
            }
        } else {
            shooter.setManualMode(false);

            // Machine Learning: Bumpers for top, D-Pad for bottom (±1%)
            if (gp2AdjustTimer.seconds() > ADJUST_DELAY) {
                if (gamepad2.left_bumper) {
                    shooter.decreaseTopPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.right_bumper) {
                    shooter.increaseTopPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_down) {
                    shooter.decreaseBottomPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_up) {
                    shooter.increaseBottomPower();
                    gp2AdjustTimer.reset();
                }
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
                break;

            case OFF:
            default:
                intake.stop();
                break;
        }
    }

    // ==================== LIGHTING ====================
    private void updateLighting() {
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

        lighting.setJamLight(intake.isJammed() ?
                LightingSubsystem.LightMode.RED :
                LightingSubsystem.LightMode.GREEN);

        lighting.update();
    }

    // ==================== TELEMETRY ====================
    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ ULTIMATE TELEOP RED ═══╗");
        telemetry.addData("│ Mode", currentMode);
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Goal Track", drivetrain.isGoalTrackingEnabled() ? "ON" : "OFF");

        telemetry.addLine("╠═══ POSITION ═══╣");
        telemetry.addData("│ STATUS", drivetrain.isHolding() ? "LOCKED (HOLD)" : "MANUAL DRIVE");
        telemetry.addData("│ X / Y", "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ Auto-Pos", autoPosition.isActive() ? "ACTIVE" : "Manual");

        telemetry.addLine("╠═══ INTAKE ═══╣");
        telemetry.addData("│ Jammed", intake.isJammed() ? "YES" : "NO");
        telemetry.addData("│ Distance", "%.1f cm", intake.getDistance());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ Status", shooter.isEnabled() ?
                (shooter.isManualMode() ? "MANUAL" : shooter.getModeName()) : "OFF");
        telemetry.addData("│ Top Power", "%.3f", shooter.getTopPower());
        telemetry.addData("│ Bottom Power", "%.3f", shooter.getBottomPower());
        telemetry.addData("│ Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("│ Current RPM", "%.0f", shooter.getCurrentRPM());
        telemetry.addData("│ At Target", shooter.isAtTarget() ? "YES" : "NO");

        if (shooter.isManualMode()) {
            telemetry.addData("│ Manual Top", "%.3f", shooter.getManualTopPower());
            telemetry.addData("│ Manual Bottom", "%.3f", shooter.getManualBottomPower());
        }

        telemetry.addLine("╠═══ LIMELIGHT ═══╣");
        telemetry.addData("│ Distance", "%.1f in", limelight.getDistanceFromShoot());
        telemetry.addData("│ Angle", "%.1f°", limelight.getAngleFromShoot());
        telemetry.addData("│ Last Update", "%.1fs ago", runtime.seconds() - limelight.getLastUpdateTime());

        telemetry.addLine("╠═══ ARTIFACT FETCHER ═══╣");
        telemetry.addData("│ Active", artifactFetcher.isActive() ? "YES" : "NO");
        telemetry.addData("│ Target Found", artifactFetcher.hasTarget() ? "YES" : "NO");

        telemetry.addLine("╚════════════════════════╝");
        telemetry.update();
    }

    // ==================== HELPER METHODS ====================
    private Pose getStartPoseFromAuto() {
        try {
            Pose bluePose = ball12blueClose.autoEndPose;
            if (bluePose != null && (bluePose.getX() != 0 || bluePose.getY() != 0)) {
                return bluePose.mirror();
            }
        } catch (Exception e) {
            // No auto end pose found
        }
        return new Pose(48.9, 66.6, Math.toRadians(180));
    }
}