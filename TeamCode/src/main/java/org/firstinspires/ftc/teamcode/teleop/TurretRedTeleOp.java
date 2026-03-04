package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.AutoPositionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Turret Red", group = "Competition")
public class TurretRedTeleOp extends OpMode {

    private static final boolean IS_RED = true;

    // =========================================================================
    //  TURRET CONSTANTS  — change MAX_ANGLE here to adjust range anytime
    // =========================================================================
    /**
     * Physical range of the turret servo in degrees.
     * ±50 means the servo can reach 50° left and 50° right of center.
     * Formula: servoPos = 0.5 + (angleDeg / MAX_TURRET_ANGLE) * 0.5
     * Change this one number to adjust the full range.
     */
    private static final double TURRET_MAX_ANGLE        = 50.0;   // degrees — tune for your servo
    private static final double TURRET_CENTER_POS       = 0.5;    // servo center position
    private static final double TURRET_ALIGN_TOLERANCE  = 2.0;    // degrees — "aligned" threshold

    /**
     * P gain for turret odom tracking PD loop.
     * Higher = snappier, too high = jitter/oscillation on the servo.
     */
    private static final double TURRET_P                = 0.012;  // servo units per degree error
    private static final double TURRET_D                = 0.001;  // derivative damping

    // Goal positions (inches) — same as SWM and Turret subsystem
    private static final double GOAL_X = IS_RED ? 144.0 : 0.0;
    private static final double GOAL_Y = 144.0;

    // =========================================================================
    //  DRIVETRAIN / FIELD-CENTRIC
    // =========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU     imu;
    private double  fieldCentricOffset = 0.0;

    private static final boolean FIELD_CENTRIC  = true;
    private double speedMultiplier              = 0.7;
    private static final double MIN_SPEED       = 0.1;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    // =========================================================================
    //  SWM HEADING CORRECTION (drivetrain) — same as NoTurretRedTeleOp
    // =========================================================================
    private static final double SWM_HEADING_P   = 2.0;
    private static final double SWM_HEADING_D   = 0.05;
    private static final double SWM_MAX_TURN    = 0.6;
    private double              swmLastError    = 0.0;
    private final ElapsedTime   swmPidTimer     = new ElapsedTime();

    // =========================================================================
    //  SUBSYSTEMS
    // =========================================================================
    private Drivetrain            drivetrain;
    private Intake                intake;
    private Shooter               shooter;
    private Limelight             limelight;
    private ShootingWhileMoving   swm;
    private AutoPositionSubsystem autoPos;
    private Turret                turret;
    private Follower              follower;

    private final ElapsedTime runtime = new ElapsedTime();

    // =========================================================================
    //  TURRET STATE
    // =========================================================================
    /**
     * When true, turret tracks the goal via odometry PD loop (GP2 L2 toggle).
     * When false, turret holds TURRET_CENTER_POS (center).
     */
    private boolean turretOdomTracking = false;

    // PD state for turret odom tracking
    private double            turretLastError = 0.0;
    private final ElapsedTime turretPidTimer  = new ElapsedTime();

    // Current commanded servo position — always write through this so we
    // never have multiple places fighting over the servo.
    private double turretServoPos = TURRET_CENTER_POS;

    // =========================================================================
    //  GP1 EDGE DETECTION
    // =========================================================================
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

    // =========================================================================
    //  GP2 EDGE DETECTION
    // =========================================================================
    private boolean lastGP2L2        = false;   // turret odom tracking toggle
    private boolean lastGP2R2        = false;
    private boolean lastGP2DpadUp    = false;
    private boolean lastGP2DpadDown  = false;
    private boolean lastGP2DpadLeft  = false;
    private boolean lastGP2DpadRight = false;
    private boolean lastGP2X         = false;
    private boolean lastGP2Triangle  = false;
    private boolean lastGP2Square    = false;
    private boolean lastGP2Circle    = false;
    private boolean lastGP2L1        = false;
    private boolean lastGP2R1        = false;
    private boolean lastGP2Share     = false;

    private boolean gateCycleActive = false;

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================

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
        turret     = new Turret(hardwareMap, limelight, IS_RED);
        swm        = new ShootingWhileMoving(follower, shooter, turret, IS_RED);
        autoPos    = new AutoPositionSubsystem(follower, IS_RED);

        telemetry.addData("Status",    "Turret Red - Ready");
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

        turretServoPos      = TURRET_CENTER_POS;
        turretOdomTracking  = false;
        turretLastError     = 0.0;
        turretPidTimer.reset();

        swmLastError = 0.0;
        swmPidTimer.reset();

        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        autoPos.update();
        swm.update();
        limelight.updateMegaTag2Orientation(follower);

        // Update turret subsystem (needed for isAligned() / SWM integration)
        turret.update(follower.getPose());

        handleGP1Drive();
        handleGP1Alignment();
        handleGP1Shooter();
        handleGP1Intake();
        handleGP1SWM();

        handleGP2TurretTracking();   // NEW — L2 toggle
        handleGP2Shoot();
        handleGP2Positioning();
        handleGP2Shooter();
        handleGP2Localization();

        // Auto RPM based on distance (future pose if SWM on)
        if (shooter.isActive()) {
            shooter.setRPMForDistance(swm.getDistanceForRPM() * 0.0254);
        }

        // Turret output — one write per loop, priority:
        //   1. Odom tracking active → PD servo output
        //   2. Tracking off → center
        applyTurretOutput();

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

    // =========================================================================
    //  TURRET OUTPUT
    // =========================================================================

    /**
     * Computes and writes the final servo position every loop.
     *
     * When odom tracking is ON:
     *   1. Compute global angle from robot to goal using atan2.
     *   2. Subtract robot heading to get robot-relative turret angle.
     *   3. Clamp to ±TURRET_MAX_ANGLE.
     *   4. Run a PD loop to smooth the servo motion.
     *   5. Map angle to [0,1] servo position.
     *
     * When odom tracking is OFF: hold center (TURRET_CENTER_POS).
     *
     * To change the turret range later:  change TURRET_MAX_ANGLE.
     * To change tracking aggression:     change TURRET_P / TURRET_D.
     * To change alignment sensitivity:   change TURRET_ALIGN_TOLERANCE.
     */
    private void applyTurretOutput() {
        if (turretOdomTracking) {
            turretServoPos = calculateTurretOdomPosition();
        } else {
            turretServoPos = TURRET_CENTER_POS;
            turretLastError = 0.0;  // reset so there's no jump when re-enabling
            turretPidTimer.reset();
        }

        // Clamp to valid servo range
        turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));

        // Write to the turret subsystem's servo via the existing setServoAngle path.
        // We bypass Turret.update() output and write the servo directly here so
        // the teleop has full priority. Alternatively, call turret.setMode(MANUAL)
        // and turret.setManualAngle() if you prefer routing through the subsystem.
        //
        // Direct approach (simplest):
        turret.setManualAngle(servoPositionToAngle(turretServoPos));
        turret.setMode(turretOdomTracking ? Turret.Mode.ODOMETRY : Turret.Mode.MANUAL);
    }

    /**
     * PD controller: computes target servo position using odometry.
     *
     * Error = desired turret angle (robot-relative) - current turret angle.
     * Output is clamped so the servo can't slam to an extreme in one loop.
     */
    private double calculateTurretOdomPosition() {
        Pose pose = follower.getPose();

        // Step 1: global angle from robot to goal
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double globalAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

        // Step 2: robot-relative angle = global angle minus robot heading
        double robotHeading = Math.toDegrees(pose.getHeading());
        double desiredTurretAngle = globalAngleToGoal - robotHeading;

        // Normalize to ±180
        while (desiredTurretAngle >  180) desiredTurretAngle -= 360;
        while (desiredTurretAngle < -180) desiredTurretAngle += 360;

        // Clamp to physical range
        desiredTurretAngle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, desiredTurretAngle));

        // Step 3: current servo angle (back-calculate from position)
        double currentAngle = servoPositionToAngle(turretServoPos);

        // Step 4: PD
        double error      = desiredTurretAngle - currentAngle;
        double dt         = turretPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - turretLastError) / dt : 0.0;
        double output     = (error * TURRET_P) + (derivative * TURRET_D);

        turretLastError = error;
        turretPidTimer.reset();

        // Step 5: apply output to current position (incremental PD)
        return turretServoPos + output;
    }

    /**
     * Convert servo position [0,1] back to angle in degrees.
     * Inverse of: pos = 0.5 + (angle / MAX) * 0.5
     */
    private double servoPositionToAngle(double pos) {
        return (pos - TURRET_CENTER_POS) / 0.5 * TURRET_MAX_ANGLE;
    }

    /**
     * Convert angle in degrees to servo position [0,1].
     */
    private double angleToDegServoPos(double angleDeg) {
        return TURRET_CENTER_POS + (angleDeg / TURRET_MAX_ANGLE) * 0.5;
    }

    /** True if turret is within TURRET_ALIGN_TOLERANCE of desired angle. */
    private boolean isTurretAligned() {
        if (!turretOdomTracking) return false;
        Pose   pose              = follower.getPose();
        double dx                = GOAL_X - pose.getX();
        double dy                = GOAL_Y - pose.getY();
        double globalAngle       = Math.toDegrees(Math.atan2(dy, dx));
        double desiredAngle      = globalAngle - Math.toDegrees(pose.getHeading());
        while (desiredAngle >  180) desiredAngle -= 360;
        while (desiredAngle < -180) desiredAngle += 360;
        desiredAngle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, desiredAngle));
        double currentAngle = servoPositionToAngle(turretServoPos);
        return Math.abs(desiredAngle - currentAngle) < TURRET_ALIGN_TOLERANCE;
    }

    // =========================================================================
    //  DRIVE HANDLING — same as NoTurretRedTeleOp
    // =========================================================================

    private void handleGP1Drive() {
        if (autoPos.isActive()) {
            boolean moving = Math.abs(gamepad1.left_stick_x)  > 0.1
                    || Math.abs(gamepad1.left_stick_y)  > 0.1
                    || Math.abs(gamepad1.right_stick_x) > 0.1;
            if (moving) autoPos.cancel();
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

        // Turn priority (same as NoTurretRedTeleOp):
        // 1. Driver manually turning → honour directly, reset SWM PD
        // 2. SWM enabled → SWM future-pose PD correction
        // 3. Drivetrain goal tracking → existing drivetrain PD
        // 4. None → rx = 0
        if (driverTurning) {
            swmLastError = 0.0;
            swmPidTimer.reset();
        } else if (swm.isEnabled() && swm.isHeadingLockActive()) {
            rx = calculateSWMTurnCorrection();
        } else if (drivetrain.isGoalTrackingEnabled()) {
            rx = drivetrain.getGoalTrackingTurn();
        }

        follower.setTeleOpDrive(y * speedMultiplier, -x * speedMultiplier, -rx * speedMultiplier, true);
    }

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
        return -turn;
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // =========================================================================
    //  GP1 HANDLERS — identical to NoTurretRedTeleOp
    // =========================================================================

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
            swmLastError = 0.0;
            swmPidTimer.reset();
            gamepad1.rumble(swm.isEnabled() ? 500 : 200);
        }
        lastOptions = gamepad1.options;
    }

    // =========================================================================
    //  GP2 HANDLERS
    // =========================================================================

    /**
     * GP2 L2 — Toggle turret odom tracking.
     *
     * First press:  turret starts tracking the goal using odometry PD.
     * Second press: turret returns to center (TURRET_CENTER_POS) and holds.
     *
     * This is exactly like the drivetrain goal tracking, but for the servo.
     * The turret doesn't fight the driver — it's a toggle, not hold-to-track.
     */
    private void handleGP2TurretTracking() {
        boolean l2 = gamepad2.left_trigger > 0.5;

        if (l2 && !lastGP2L2) {
            turretOdomTracking = !turretOdomTracking;
            turretLastError    = 0.0;
            turretPidTimer.reset();

            if (turretOdomTracking) {
                gamepad2.rumble(500);   // long buzz = tracking ON
            } else {
                turretServoPos = TURRET_CENTER_POS;  // snap back to center
                gamepad2.rumble(200);                // short buzz = tracking OFF / centered
            }
        }
        lastGP2L2 = l2;
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

        if (gateCycleActive) intake.setMode(Intake.Mode.INTAKE);
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

    // =========================================================================
    //  TELEMETRY
    // =========================================================================

    private void displayTelemetry() {
        Pose   pose       = follower.getPose();
        Pose   futurePose = swm.getFuturePose();
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset);
        double currentTurretAngle = servoPositionToAngle(turretServoPos);

        telemetry.addLine("╔═══ TURRET RED ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Heading", "%.1f°  (R3 to reset)", headingDeg);
        telemetry.addData("│ AutoPos", autoPos.isActive() ? "ACTIVE" : "idle");

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X / Y",   "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Tracking", turretOdomTracking ? "ODOM ON ✓" : "centered");
        telemetry.addData("│ Angle",    "%.1f°  (max ±%.0f°)", currentTurretAngle, TURRET_MAX_ANGLE);
        telemetry.addData("│ ServoPos", "%.3f", turretServoPos);
        telemetry.addData("│ Aligned",  isTurretAligned() ? "YES ✓" : "NO");
        telemetry.addData("│ [GP2 L2]", "Toggle odom tracking");

        telemetry.addLine("╠═══ DRIVETRAIN ALIGN ═══╣");
        telemetry.addData("│ Mode",    drivetrain.isGoalTrackingEnabled()
                ? (drivetrain.isVisionAssistEnabled() ? "LIMELIGHT" : "ODOMETRY") : "OFF");
        telemetry.addData("│ Aligned", drivetrain.isAlignedWithGoal() ? "YES ✓" : "NO");
        telemetry.addData("│ TX",      "%.1f°", limelight.getTx());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled",      swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Heading Lock", swm.isHeadingLockActive() ? "ACTIVE" : "waiting");
        telemetry.addData("│ Ready",        swm.isReadyToShoot() ? "SHOOT NOW ✓" : "Waiting");
        telemetry.addData("│ Distance",     "%.1f in", swm.getDistanceForRPM());
        telemetry.addData("│ Target Hdg",   "%.1f°", Math.toDegrees(swm.getTargetHeading()));
        telemetry.addData("│ Future X/Y",   "%.1f, %.1f", futurePose.getX(), futurePose.getY());
        telemetry.addData("│ Hdg Err",      "%.2f°", Math.toDegrees(
                normalizeAngle(swm.getTargetHeading() - pose.getHeading())));
        telemetry.addData("│ Velocity",     "%.1f in/s", swm.getVelocityMagnitude());
        telemetry.addData("│ Accel X/Y",    "%.1f, %.1f", swm.getAccelX(), swm.getAccelY());

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }
}