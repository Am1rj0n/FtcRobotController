package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.AutoPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "No Turret Blue", group = "Competition")
public class NoTurretBlueTeleOp extends OpMode {

    private static final boolean IS_RED        = false;
    private static final double  TURRET_CENTER = 0.5;

    // Set to true to enable field-centric
    private static final boolean FIELD_CENTRIC = false;

    private Drivetrain            drivetrain;
    private Intake                intake;
    private Shooter               shooter;
    private Limelight             limelight;
    private ShootingWhileMoving   swm;
    private AutoPositionSubsystem autoPos;
    private Servo                 turretServo;
    private Follower              follower;

    private final ElapsedTime runtime = new ElapsedTime();

    // GP1 debounce
    private boolean lastCircle   = false;
    private boolean lastCross    = false;
    private boolean lastSquare   = false;
    private boolean lastTriangle = false;
    private boolean lastL2       = false;
    private boolean lastR2       = false;
    private boolean lastR3       = false;
    private boolean lastTouchpad = false;
    private boolean lastOptions  = false;

    // GP2 debounce
    private boolean lastGP2R2       = false;
    private boolean lastGP2DpadUp   = false;
    private boolean lastGP2DpadDown = false;
    private boolean lastGP2DpadLeft = false;
    private boolean lastGP2X        = false;
    private boolean lastGP2Triangle = false;
    private boolean lastGP2Square   = false;
    private boolean lastGP2Circle   = false;
    private boolean lastGP2L1       = false;
    private boolean lastGP2R1       = false;
    private boolean lastGP2Share    = false;

    @Override
    public void init() {
        follower   = Constants.createFollower(hardwareMap);
        limelight  = new Limelight(hardwareMap, IS_RED);
        drivetrain = new Drivetrain(hardwareMap, follower, IS_RED);
        drivetrain.setLimelight(limelight);
        intake     = new Intake(hardwareMap);
        shooter    = new Shooter(hardwareMap);
        swm        = new ShootingWhileMoving(follower, shooter, null, IS_RED);
        autoPos    = new AutoPositionSubsystem(follower, IS_RED);

        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(TURRET_CENTER);

        telemetry.addData("Status",    "No Turret Blue - Ready");
        telemetry.addData("Auto Pose", AutoToTeleTransfer.finalPose != null ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void start() {
        if (AutoToTeleTransfer.finalPose != null) {
            follower.setStartingPose(AutoToTeleTransfer.finalPose);
            gamepad1.rumble(500);
        } else {
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        }
        follower.startTeleopDrive();
        limelight.start();
        turretServo.setPosition(TURRET_CENTER);
        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        autoPos.update();
        swm.update();
        limelight.updateMegaTag2Orientation(follower);

        handleGP1Drive();
        handleGP1Alignment();
        handleGP1Shooter();
        handleGP1Intake();
        handleGP1SWM();
        handleGP2Shoot();
        handleGP2Positioning();
        handleGP2Shooter();
        handleGP2Localization();

        if (swm.isHeadingLockActive()) {
            drivetrain.setHeadingLock(swm.getTargetHeading());
        } else {
            drivetrain.releaseHeadingLock();
        }

        if (shooter.isActive()) {
            shooter.setRPMForDistance(swm.getDistanceForRPM() * 0.0254);
        }

        turretServo.setPosition(TURRET_CENTER);
        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        drivetrain.stop();
        limelight.stop();
        autoPos.cancel();
    }

    private void handleGP1Drive() {
        if (autoPos.isActive()) {
            boolean moving = Math.abs(gamepad1.left_stick_x)  > 0.1
                    || Math.abs(gamepad1.left_stick_y)  > 0.1
                    || Math.abs(gamepad1.right_stick_x) > 0.1;
            if (moving) autoPos.cancel();
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            if (drivetrain.isHolding()) gamepad1.rumble(500);
        }
        lastTouchpad = gamepad1.touchpad;

        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        if (gamepad1.left_bumper)  drivetrain.decreaseSpeed();
        if (gamepad1.right_bumper) drivetrain.increaseSpeed();

        if (gamepad1.right_stick_button && !lastR3) {
            drivetrain.resetFieldCentric();
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        double speed = drivetrain.getSpeed();

        if (!FIELD_CENTRIC) {
            // ── ROBOT CENTRIC (active) ──────────────────────────────────────
            follower.setTeleOpDrive(
                    forward * speed,
                    strafe  * speed,
                    turn    * speed,
                    true
            );
        } else {
            // ── FIELD CENTRIC (set FIELD_CENTRIC = true to use) ────────────
            drivetrain.drive(forward, strafe, turn);
        }
    }

    private void handleGP1Alignment() {
        boolean l2 = gamepad1.left_trigger  > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) {
            drivetrain.enableGoalTracking(true);  // Limelight
            gamepad1.rumble(200);
        }
        if (r2 && !lastR2) {
            drivetrain.enableGoalTracking(false); // Odometry
            gamepad1.rumble(200);
        }
        if (!l2 && !r2 && (lastL2 || lastR2)) {
            drivetrain.disableGoalTracking();
        }

        lastL2 = l2;
        lastR2 = r2;
    }

    private void handleGP1Shooter() {
        if (gamepad1.circle && !lastCircle) {
            shooter.toggle();
            gamepad1.rumble(shooter.isActive() ? 500 : 200);
        }
        lastCircle = gamepad1.circle;
    }

    private void handleGP1Intake() {
        if (gamepad1.cross     && !lastCross)    intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.square    && !lastSquare)   intake.setMode(Intake.Mode.OFF);
        if (gamepad1.triangle  && !lastTriangle) intake.setMode(Intake.Mode.SPIT);

        lastCross    = gamepad1.cross;
        lastSquare   = gamepad1.square;
        lastTriangle = gamepad1.triangle;
    }

    private void handleGP1SWM() {
        if (gamepad1.options && !lastOptions) {
            swm.toggle();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;
    }

    private void handleGP2Shoot() {
        boolean r2 = gamepad2.right_trigger > 0.5;
        if (r2  && !lastGP2R2) intake.setMode(Intake.Mode.SHOOT);
        if (!r2 &&  lastGP2R2) intake.setMode(Intake.Mode.OFF);
        lastGP2R2 = r2;
    }

    private void handleGP2Positioning() {
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            autoPos.goToCloseShoot();
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2DpadDown) {
            autoPos.goToFarShoot();
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastGP2DpadLeft) {
            autoPos.goToPark();
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadLeft = gamepad2.dpad_left;

        if (gamepad2.cross && !lastGP2X) {
            autoPos.cancel();
            gamepad2.rumbleBlips(2);
        }
        lastGP2X = gamepad2.cross;
    }

    private void handleGP2Shooter() {
        if (gamepad2.triangle && !lastGP2Triangle) {
            if (shooter.getRPMMode() == Shooter.RPMMode.FAR) shooter.setCloseMode();
            else shooter.setFarMode();
            gamepad2.rumbleBlips(1);
        }
        lastGP2Triangle = gamepad2.triangle;

        if (gamepad2.square && !lastGP2Square) {
            shooter.setAutoMode();
            gamepad2.rumbleBlips(1);
        }
        lastGP2Square = gamepad2.square;

        if (gamepad2.circle && !lastGP2Circle) {
            shooter.setManualFromCurrent();
            gamepad2.rumbleBlips(1);
        }
        lastGP2Circle = gamepad2.circle;

        if (gamepad2.left_bumper  && !lastGP2L1) shooter.decreaseRPM();
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    private void handleGP2Localization() {
        if (gamepad2.share && !lastGP2Share) {
            boolean success = limelight.megaTag2Localize(follower);
            if (success) { gamepad2.rumble(1000); gamepad1.rumble(500); }
            else          { gamepad2.rumbleBlips(3); }
        }
        lastGP2Share = gamepad2.share;
    }

    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ NO TURRET BLUE ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "%.0f%%", drivetrain.getSpeed() * 100);
        telemetry.addData("│ Drive",   FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("│ AutoPos", autoPos.isActive() ? "ACTIVE" : "idle");

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X / Y",   "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ ALIGNMENT ═══╣");
        telemetry.addData("│ Mode",    drivetrain.isGoalTrackingEnabled()
                ? (drivetrain.isVisionAssistEnabled() ? "LIMELIGHT" : "ODOMETRY") : "OFF");
        telemetry.addData("│ Aligned", drivetrain.isAlignedWithGoal() ? "YES ✓" : "NO");
        telemetry.addData("│ TX",      "%.1f°", limelight.getTx());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled",      swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Heading Lock", swm.isHeadingLockActive() ? "ACTIVE" : "Off");
        telemetry.addData("│ Ready",        swm.isReadyToShoot() ? "SHOOT NOW" : "Waiting");
        telemetry.addData("│ Distance",     "%.1f in", swm.getDistanceForRPM());

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }
}