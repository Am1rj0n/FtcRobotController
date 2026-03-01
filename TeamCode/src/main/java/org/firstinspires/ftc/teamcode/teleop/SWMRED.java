package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "No Turret TEST red", group = "Competition")
public class SWMRED extends OpMode {

    private static final boolean IS_RED        = true;
    private static final double  TURRET_CENTER = 0.5;

    // GM0 field-centric - direct motor control
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU     imu;
    private double  fieldCentricOffset = 0.0;

    private static final boolean FIELD_CENTRIC  = true;
    private double speedMultiplier = 0.7;
    private static final double MIN_SPEED       = 0.1;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    // =====================================================================
    //  SWM HEADING CORRECTION — tune these
    // =====================================================================
    /**
     * P gain for the SWM heading correction loop.
     * Higher = snappier alignment, too high = oscillation.
     * Start around 2.0 and tune from there.
     */
    private static final double SWM_HEADING_P   = 1.3;

    /**
     * D gain for the SWM heading correction loop.
     * Helps damp oscillation during fast rotation.
     */
    private static final double SWM_HEADING_D   = 0.05;

    /**
     * Maximum turn correction SWM can apply (0.0–1.0).
     * Prevents SWM from spinning the robot uncontrollably.
     */
    private static final double SWM_MAX_TURN    = 0.6;

    // SWM PD state
    private double      swmLastError  = 0.0;
    private final ElapsedTime swmPidTimer = new ElapsedTime();

    // Pedro / subsystems
    private Drivetrain            drivetrain;
    private Intake                intake;
    private Shooter               shooter;
    private Limelight             limelight;
    private ShootingWhileMoving   swm;
    private AutoPositionSubsystem autoPos;
    private Servo                 turretServo;
    private Follower              follower;

    private final ElapsedTime runtime = new ElapsedTime();

    // GP1 edge detection
    private boolean lastCircle   = false;
    private boolean lastCross    = false;
    private boolean lastSquare   = false;
    private boolean lastTriangle = false;
    private boolean lastL1       = false;
    private boolean lastR1       = false;
    private boolean lastL2       = false;
    private boolean lastR2       = false;
    private boolean lastR3       = false;
    private boolean lastTouchpad = false;
    private boolean lastOptions  = false;

    // GP2 edge detection
    private boolean lastGP1R2        = false;
    private boolean lastGP2DpadUp    = false;
    private boolean lastGP2DpadDown  = false;
    private boolean lastGP2DpadLeft  = false;
    private boolean lastGP2X         = false;
    private boolean lastGP2Triangle  = false;
    private boolean lastGP2Square    = false;
    private boolean lastGP2Circle    = false;
    private boolean lastGP2L1        = false;
    private boolean lastGP2R1        = false;
    private boolean lastGP2Share     = false;
    private boolean lastGP2DpadRight = false;

    private boolean gateCycleActive = false;

    // =====================================================================
    //  LIFECYCLE
    // =====================================================================

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

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

        imu.resetYaw();
        fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        limelight.start();
        turretServo.setPosition(TURRET_CENTER);

        swmLastError = 0.0;
        swmPidTimer.reset();

        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        autoPos.update();
        swm.update();  // always update so future pose + accel stay fresh
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
        limelight.stop();
        autoPos.cancel();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // =====================================================================
    //  DRIVE HANDLING
    // =====================================================================

    private void handleGP1Drive() {
        if (autoPos.isActive()) {
            boolean moving = Math.abs(gamepad1.left_stick_x)  > 0.1
                    || Math.abs(gamepad1.left_stick_y)  > 0.1
                    || Math.abs(gamepad1.right_stick_x) > 0.1;
            if (moving) {
                autoPos.cancel();
            }
        }

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.left_bumper && !lastL1)
            speedMultiplier = Math.max(MIN_SPEED, speedMultiplier - SPEED_INCREMENT);
        if (gamepad1.right_bumper && !lastR1)
            speedMultiplier = Math.min(MAX_SPEED, speedMultiplier + SPEED_INCREMENT);
        lastL1 = gamepad1.left_bumper;
        lastR1 = gamepad1.right_bumper;

        if (gamepad1.right_stick_button && !lastR3) {
            fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            gamepad1.rumble(200);
        }
        lastR3 = gamepad1.right_stick_button;

        if (gamepad1.touchpad && !lastTouchpad) {
            drivetrain.toggleHold();
            if (drivetrain.isHolding()) gamepad1.rumble(500);
        }
        lastTouchpad = gamepad1.touchpad;

        if (gamepad1.share) {
            drivetrain.resetToCorner();
            gamepad1.rumbleBlips(2);
        }

        boolean driverTurning = Math.abs(rx) > 0.05;

        // ── Turn correction priority ──────────────────────────────────────
        // 1. Driver is manually turning → honour it directly, reset SWM PD
        // 2. SWM is enabled → apply SWM future-pose heading PD correction
        // 3. Odom goal tracking is enabled → apply drivetrain goal-tracking PD
        // 4. Nothing → pass rx through as-is (0 if stick is neutral)
        // ─────────────────────────────────────────────────────────────────
        if (driverTurning) {
            // Driver has the wheel — reset integrators so we don't get a
            // jump when they release the stick
            swmLastError = 0.0;
            swmPidTimer.reset();
            // rx already set from stick, fall through to setTeleOpDrive
        } else if (swm.isEnabled() && swm.isHeadingLockActive()) {
            rx = calculateSWMTurnCorrection();
        } else if (drivetrain.isGoalTrackingEnabled()) {
            rx = drivetrain.getGoalTrackingTurn();
        }
        // else rx stays as-is (0.0 from neutral stick)

        follower.setTeleOpDrive(y * speedMultiplier, -x * speedMultiplier, -rx * speedMultiplier, true);
    }

    /**
     * PD controller that steers the drivetrain toward the SWM target heading
     * (heading aimed at future predicted goal intersection, not current pose).
     *
     * Returns a turn value in [-1, 1] ready to feed into setTeleOpDrive.
     */
    private double calculateSWMTurnCorrection() {
        double currentHeading = follower.getPose().getHeading();
        double targetHeading  = swm.getTargetHeading();
        double error          = normalizeAngle(targetHeading - currentHeading);

        double dt         = swmPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - swmLastError) / dt : 0.0;

        double turn = (error * SWM_HEADING_P) + (derivative * SWM_HEADING_D);
        turn = Math.max(-SWM_MAX_TURN, Math.min(SWM_MAX_TURN, turn));

        swmLastError = error;
        swmPidTimer.reset();

        // Negate: Pedro setTeleOpDrive convention — positive rx = CCW,
        // our error is CCW-positive, so we negate to get CW correction.
        return -turn;
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // =====================================================================
    //  OTHER HANDLERS (unchanged logic, same as before)
    // =====================================================================

    private void handleGP1Alignment() {
        boolean l2 = gamepad1.left_trigger  > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;

        if (l2 && !lastL2) { drivetrain.enableGoalTracking(true);  gamepad1.rumble(200); }
        if (r2 && !lastR2) { drivetrain.enableGoalTracking(false); gamepad1.rumble(200); }
        if (!l2 && !r2 && (lastL2 || lastR2)) drivetrain.disableGoalTracking();

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
            // Reset PD state on toggle so there's no heading error spike
            swmLastError = 0.0;
            swmPidTimer.reset();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;
    }

    private void handleGP2Shoot() {
        boolean r2 = gamepad1.right_trigger > 0.5;
        if (r2  && !lastGP1R2) intake.setMode(Intake.Mode.SHOOT);
        if (!r2 &&  lastGP1R2) intake.setMode(Intake.Mode.OFF);
        lastGP1R2 = r2;
    }

    private void handleGP2Positioning() {
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            autoPos.goToCloseShoot();
            shooter.setCloseMode();
            gateCycleActive = false;
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2DpadDown) {
            autoPos.goToFarShoot();
            shooter.setFarMode();
            gateCycleActive = false;
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastGP2DpadLeft) {
            autoPos.goToPark();
            gateCycleActive = false;
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadLeft = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastGP2DpadRight) {
            autoPos.goToGateCycle();
            gateCycleActive = true;
            intake.setMode(Intake.Mode.INTAKE);
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadRight = gamepad2.dpad_right;

        if (gamepad2.cross && !lastGP2X) {
            autoPos.cancel();
            gateCycleActive = false;
            gamepad2.rumbleBlips(2);
        }
        lastGP2X = gamepad2.cross;

        if (gateCycleActive) {
            intake.setMode(Intake.Mode.INTAKE);
        }
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

    // =====================================================================
    //  TELEMETRY
    // =====================================================================

    private void displayTelemetry() {
        Pose   pose       = follower.getPose();
        Pose   futurePose = swm.getFuturePose();
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset);

        telemetry.addLine("╔═══ NO TURRET BLUE ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Drive",   FIELD_CENTRIC ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("│ Heading", "%.1f°  (R3 to reset)", headingDeg);
        telemetry.addData("│ AutoPos", autoPos.isActive() ? "ACTIVE" : "idle");

        telemetry.addLine("╠═══ POSE (Odometry) ═══╣");
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
        telemetry.addData("│ Enabled",       swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Heading Lock",  swm.isHeadingLockActive() ? "ACTIVE" : "waiting");
        telemetry.addData("│ Ready",         swm.isReadyToShoot() ? "SHOOT NOW ✓" : "Waiting");
        telemetry.addData("│ Distance",      "%.1f in", swm.getDistanceForRPM());
        telemetry.addData("│ Target Hdg",    "%.1f°", Math.toDegrees(swm.getTargetHeading()));
        telemetry.addData("│ Future X/Y",    "%.1f, %.1f", futurePose.getX(), futurePose.getY());
        telemetry.addData("│ Heading Err",   "%.2f°", Math.toDegrees(
                normalizeAngle(swm.getTargetHeading() - pose.getHeading())));
        telemetry.addData("│ Velocity",      "%.1f in/s", swm.getVelocityMagnitude());
        telemetry.addData("│ Accel X/Y",     "%.1f, %.1f", swm.getAccelX(), swm.getAccelY());

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }
}