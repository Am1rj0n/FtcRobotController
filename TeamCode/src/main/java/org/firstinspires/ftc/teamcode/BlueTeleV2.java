package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subfilesV2.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "TeleOP Blue Main", group = "TeleOp")
public class BlueTeleV2 extends OpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private shootersubv2 shooter;
    private LightingSubsystem lighting;
    private AutoPositionSubsystem autoPosition;
    private LimelightHelper limelight;
    private ArtifactFetcher artifactFetcher;

    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    // Alliance
    private static final boolean IS_RED = false;

    // System states
    private enum SystemMode {OFF, INTAKE, SPIT, SHOOT}

    private SystemMode currentMode = SystemMode.OFF;

    // Debouncing
    private boolean lastSquare = false;
    private boolean lastCross = false;
    private boolean lastTriangle = false;
    private boolean lastR2 = false;
    private boolean lastGP1R2 = false;
    private boolean lastGP2Triangle = false;
    private boolean lastLeftTrigger = false;
    private boolean lastTouchpad = false;
    private boolean lastCircle = false;
    private boolean lastR3 = false;
    private boolean lastGP2Cross = false;
    private boolean lastDpadUp = false;  // ← NEW: For vision assist toggle

    // Time-based debouncing for shooter adjustments
    private ElapsedTime gp2AdjustTimer = new ElapsedTime();
    private static final double ADJUST_DELAY = 0.15;

    // Store current power values for telemetry
    private double currentTopPower = 0.0;
    private double currentBottomPower = 0.0;

    @Override
    public void init() {
        // Initialize Pedro follower
        follower = Constants.createFollower(hardwareMap);

        // Try to get pose from autonomous
        Pose startPose = getStartPoseFromAuto();
        follower.setStartingPose(startPose);

        // Initialize Limelight FIRST
        limelight = new LimelightHelper(hardwareMap, IS_RED);

        // Initialize all subsystems (drivetrain needs limelight now)
        drivetrain = new DrivetrainSubsystem(hardwareMap, follower, IS_RED, limelight);  // ← ADDED limelight parameter
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new shootersubv2(hardwareMap, IS_RED);
        lighting = new LightingSubsystem(hardwareMap);
        autoPosition = new AutoPositionSubsystem(follower, IS_RED);
        artifactFetcher = new ArtifactFetcher(hardwareMap, follower);

        telemetry.addLine("✓ Ultimate TeleOp BLUE Initialized");
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
        follower.update();
        limelight.updateLocalization(follower, runtime.seconds());

        handleModeSelection();
        handleAutoPositioning();
        handleArtifactFetcher();

        // ONLY run manual drive if the fetcher isn't active
        if (!artifactFetcher.isActive()) {
            handleDrive();
        }

        handleShooterControls();
        updateShooterMotors();
        applySystemMode();

        updateLighting();
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
            if (drivetrain.isHolding()) {
                gamepad1.rumble(500);
            }
        }
        lastTouchpad = gamepad1.touchpad;

        // --- SHARE: CORNER RESET ---
        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        // Speed control with bumpers (L1/R1)
        if (gamepad1.left_bumper) drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        // Field-centric reset on R3 (right stick button) with rumble
        boolean r3 = gamepad1.right_stick_button;
        if (r3 && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = r3;

        // R2: Goal tracking toggle (NO vibration, only light feedback)
        boolean gp1R2 = gamepad1.right_trigger > 0.5;
        if (gp1R2 && !lastGP1R2) {
            drivetrain.toggleGoalTracking();
        }
        lastGP1R2 = gp1R2;

        // --- DPAD UP: VISION ASSIST TOGGLE (NEW) ---
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !lastDpadUp) {
            boolean nowEnabled = drivetrain.toggleVisionAssist();
            if (nowEnabled) {
                gamepad1.rumble(500);  // Single long vibration when ON
            } else {
                gamepad1.rumbleBlips(2);  // Double vibration when OFF
            }
        }
        lastDpadUp = dpadUp;

        drivetrain.drive(forward, strafe, turn);
    }

    // ==================== MODE SELECTION ====================
    private void handleModeSelection() {
        // GAMEPAD 1: Face button controls
        boolean square = gamepad1.square;
        boolean cross = gamepad1.cross;
        boolean triangle = gamepad1.triangle;
        boolean circle = gamepad1.circle;

        if (square && !lastSquare) {
            currentMode = SystemMode.OFF;
            intake.resetJam();
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

        // GAMEPAD 2: X button as JAM OVERRIDE + INTAKE MODE
        boolean gp2Cross = gamepad2.cross;
        if (gp2Cross && !lastGP2Cross) {
            currentMode = SystemMode.INTAKE;
        }
        lastGP2Cross = gp2Cross;
    }

    // ==================== AUTO POSITIONING ====================
    private void handleAutoPositioning() {
        if (gamepad1.dpad_down) {
            autoPosition.goToFarShoot();
        }

        if (gamepad1.dpad_left) {
            autoPosition.goToCloseShoot();
        }

        if (gamepad1.dpad_right) {
            autoPosition.goToPark();
        }

        // Cancel auto positioning with ANY left stick movement
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
            autoPosition.cancel();
        }

        autoPosition.update();
    }

    // ==================== ARTIFACT FETCHER ====================
    private void handleArtifactFetcher() {
        boolean leftTrigger = gamepad1.left_trigger > 0.5;

        if (leftTrigger && !lastLeftTrigger) {
            artifactFetcher.toggle();
        }
        lastLeftTrigger = leftTrigger;

        if (artifactFetcher.isActive()) {
            if (Math.abs(gamepad1.left_stick_y) > 0.2 || Math.abs(gamepad1.left_stick_x) > 0.2) {
                artifactFetcher.toggle();
            } else {
                artifactFetcher.update();

                if (currentMode == SystemMode.OFF && !intake.isJammed()) {
                    currentMode = SystemMode.INTAKE;
                }
            }
        }
    }

    // ==================== SHOOTER CONTROLS ====================
    private void handleShooterControls() {
        // GAMEPAD 2 R2: Toggle shooter ON/OFF
        boolean r2 = gamepad2.right_trigger > 0.5;
        if (r2 && !lastR2) {
            shooter.toggle();
        }
        lastR2 = r2;

        // GAMEPAD 2 TRIANGLE: Toggle between Close/Far mode
        boolean gp2Triangle = gamepad2.triangle;
        if (gp2Triangle && !lastGP2Triangle) {
            shooter.toggleMode();
        }
        lastGP2Triangle = gp2Triangle;

        // GAMEPAD 2 L2 HOLD: Manual mode
        boolean manualMode = gamepad2.left_trigger > 0.1;
        shooter.setManualMode(manualMode);

        if (manualMode) {
            // L2 HELD: D-Pad adjusts BOTH top and bottom by ±5%
            if (gp2AdjustTimer.seconds() > ADJUST_DELAY) {
                if (gamepad2.dpad_up) {
                    shooter.increaseManualBothPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.dpad_down) {
                    shooter.decreaseManualBothPower();
                    gp2AdjustTimer.reset();
                }
            }
        } else {
            // L2 NOT HELD: Bumpers adjust BOTH by ±1%
            if (gp2AdjustTimer.seconds() > ADJUST_DELAY) {
                if (gamepad2.left_bumper) {
                    shooter.decreaseBothPower();
                    gp2AdjustTimer.reset();
                }
                if (gamepad2.right_bumper) {
                    shooter.increaseBothPower();
                    gp2AdjustTimer.reset();
                }
            }
        }
    }

    // ==================== SHOOTER MOTOR CONTROL ====================
    private void updateShooterMotors() {
        if (!shooter.isEnabled()) {
            shooter.getTopMotor().setPower(0);
            shooter.getBottomMotor().setPower(0);
            currentTopPower = 0.0;
            currentBottomPower = 0.0;
            return;
        }

        double targetTopPower = shooter.getTargetTopPower();
        double targetBottomPower = shooter.getTargetBottomPower();

        targetTopPower = Math.max(0, Math.min(1.0, targetTopPower));
        targetBottomPower = Math.max(0, Math.min(1.0, targetBottomPower));

        shooter.getTopMotor().setPower(targetTopPower);
        shooter.getBottomMotor().setPower(targetBottomPower);

        currentTopPower = targetTopPower;
        currentBottomPower = targetBottomPower;
    }

    // ==================== SYSTEM MODE APPLICATION ====================
    private void applySystemMode() {
        switch (currentMode) {
            case INTAKE:
                if (gamepad2.cross) {
                    intake.runIntake();
                } else if (intake.isJammed()) {
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
            if (drivetrain.isGoalTrackingEnabled()) {
                if (shooter.isCloseMode()) {
                    lighting.setShooterLight(LightingSubsystem.LightMode.PINK_BLINK);
                } else {
                    lighting.setShooterLight(LightingSubsystem.LightMode.BLUE_BLINK);
                }
            } else {
                if (shooter.isCloseMode()) {
                    lighting.setShooterLight(LightingSubsystem.LightMode.PINK);
                } else {
                    lighting.setShooterLight(LightingSubsystem.LightMode.BLUE);
                }
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

        telemetry.addLine("╔═══ ULTIMATE TELEOP BLUE ═══╗");
        telemetry.addData("│ Mode", currentMode);
        telemetry.addData("│ Speed", "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Goal Track", drivetrain.isGoalTrackingEnabled() ? "ACTIVE" : "OFF");

        telemetry.addLine("╠═══ POSITION ═══╣");
        telemetry.addData("│ STATUS", drivetrain.isHolding() ? "LOCKED (HOLD)" : "MANUAL DRIVE");
        telemetry.addData("│ X / Y", "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("│ Auto-Pos", autoPosition.isActive() ? "ACTIVE" : "Manual");

        telemetry.addLine("╠═══ INTAKE ═══╣");
        telemetry.addData("│ Jammed", intake.isJammed() ? "YES" : "NO");
        telemetry.addData("│ Distance", "%.1f cm", intake.getDistance());
        telemetry.addData("│ Override", gamepad2.cross ? "ACTIVE" : "Off");

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ Status", shooter.isEnabled() ?
                (shooter.isManualMode() ? "MANUAL" : shooter.getModeName()) : "OFF");
        telemetry.addData("│ Top Power", "%.3f", currentTopPower);
        telemetry.addData("│ Bottom Power", "%.3f", currentBottomPower);

        if (shooter.isManualMode()) {
            telemetry.addData("│ Manual Power", "%.3f", shooter.getManualBothPower());
        }

        telemetry.addLine("╠═══ LIMELIGHT ═══╣");
        telemetry.addData("│ Vision Assist", drivetrain.isVisionAssistEnabled() ? "ON ✓" : "OFF");
        telemetry.addData("│ Using", drivetrain.isUsingVision() ? "VISION ✓" : "Odometry");
        telemetry.addData("│ Distance", "%.1f in", limelight.getDistanceFromShoot());
        telemetry.addData("│ Angle", "%.1f°", limelight.getAngleFromShoot());
        telemetry.addData("│ Last Update", "%.1fs ago", runtime.seconds() - limelight.getLastUpdateTime());

        telemetry.addLine("╠═══ ARTIFACT FETCHER ═══╣");
        telemetry.addData("│ Active", artifactFetcher.isActive() ? "YES" : "NO");
        telemetry.addData("│ Target Found", artifactFetcher.hasTarget() ? "YES" : "NO");

        telemetry.addLine("╚════════════════════════╝");
        telemetry.update();
    }

    private Pose getStartPoseFromAuto() {
        try {
            Pose bluePose = org.firstinspires.ftc.teamcode.paths.robotv2.Ball12BlueFar.autoEndPose;
            if (bluePose != null && (bluePose.getX() != 0 || bluePose.getY() != 0)) {
                return bluePose;
            }
        } catch (Exception e) {
            // No auto end pose found
        }
        return new Pose(50.393, 22.331, Math.toRadians(113.5));
    }
}