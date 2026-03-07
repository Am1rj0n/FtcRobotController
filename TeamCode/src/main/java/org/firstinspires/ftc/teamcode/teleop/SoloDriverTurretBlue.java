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
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;

/**
 * SoloDriverTurretBlue — single primary gamepad + GP2 RPM fine adjust.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  GP1 CONTROLS
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  LEFT STICK          Drive (field-centric, 100% speed)
 *  RIGHT STICK         Rotate
 *  R3 (stick click)    Reset field-centric heading
 *
 *  L2 (hold)           Drivetrain odom alignment ON while held, OFF on release
 *  L1 (toggle)         Turret odom tracking (current pose → goal)
 *                        ON = long rumble, OFF = short rumble + turret centers
 *
 *  R2 (hold)           Shoot (intake → SHOOT while held, OFF on release)
 *
 *  Circle              Toggle shooter on/off
 *  Cross               Intake ON
 *  Square              Intake OFF
 *  Triangle            Intake SPIT
 *
 *  Touchpad (tap)      Toggle position hold (drivetrain holds current pose)
 *  D-pad Up            Toggle turret SWM mode (turret only — drivetrain stays free)
 *  Share               Reset odometry to corner
 *  R1 (tap)            Cycle shooter preset  CLOSE → FAR → AUTO → CLOSE
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  GP2 CONTROLS (fine adjust only — second driver optional)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  R1 (tap)            RPM increase (fine adjust)
 *  L1 (tap)            RPM decrease (fine adjust)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  LIGHTS (automatic — goBILDA RGB Indicator on "Lights" servo)
 *  FAR   → red    solid / blink when aligned
 *  CLOSE → green  solid / blink when aligned
 *  AUTO  → orange solid / blink when aligned
 *  Shooter off → light off
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "Solo Driver Turret Blue [TEST]", group = "Test")
public class SoloDriverTurretBlue extends OpMode {

    private static final boolean IS_RED = false;

    // =========================================================================
    //  TURRET CONSTANTS
    // =========================================================================
    private static final double TURRET_MAX_ANGLE       = 50.0;
    private static final double TURRET_CENTER_POS      = 0.5;
    private static final double TURRET_ALIGN_TOLERANCE = 2.0;
    private static final double TURRET_P               = 0.012;
    private static final double TURRET_D               = 0.001;

    private static final double GOAL_X = IS_RED ? 144.0 : 0.0;
    private static final double GOAL_Y = 144.0;

    // =========================================================================
    //  HARDWARE
    // =========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU     imu;
    private double  fieldCentricOffset = 0.0;

    // =========================================================================
    //  SUBSYSTEMS
    // =========================================================================
    private Drivetrain          drivetrain;
    private Intake              intake;
    private Shooter             shooter;
    private Limelight           limelight;
    private ShootingWhileMoving swm;
    private Turret              turret;
    private Lights              lights;
    private Follower            follower;

    private final ElapsedTime runtime = new ElapsedTime();

    // =========================================================================
    //  TURRET STATE
    // =========================================================================
    /** L1 toggle — turret tracks goal from current odom pose. */
    private boolean turretOdomTracking = false;

    /** D-pad Up toggle — turret tracks goal from FUTURE SWM pose. Drivetrain unaffected. */
    private boolean turretSWMMode = false;

    private double            turretLastError = 0.0;
    private final ElapsedTime turretPidTimer  = new ElapsedTime();
    private double            turretServoPos  = TURRET_CENTER_POS;

    // =========================================================================
    //  GP1 EDGE DETECTION
    // =========================================================================
    private boolean lastCircle   = false;
    private boolean lastCross    = false;
    private boolean lastSquare   = false;
    private boolean lastTriangle = false;
    private boolean lastL1       = false;  // turret odom toggle
    private boolean lastL2       = false;  // drivetrain alignment hold
    private boolean lastR1       = false;  // shooter mode cycle
    private boolean lastR2       = false;  // shoot hold
    private boolean lastR3       = false;
    private boolean lastTouchpad = false;  // position hold toggle
    private boolean lastDpadUp   = false;

    // =========================================================================
    //  GP2 EDGE DETECTION (RPM fine adjust only)
    // =========================================================================
    private boolean lastGP2R1 = false;
    private boolean lastGP2L1 = false;

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
        lights     = new Lights(hardwareMap);

        telemetry.addData("Status", "Solo Driver Turret Blue [TEST] - Ready");
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

        turretServoPos     = TURRET_CENTER_POS;
        turretOdomTracking = false;
        turretSWMMode      = false;
        turretLastError    = 0.0;
        turretPidTimer.reset();

        runtime.reset();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.periodic();
        swm.update();
        limelight.updateMegaTag2Orientation(follower);
        turret.update(follower.getPose());

        handleDrive();
        handleShooterToggle();
        handleIntake();
        handleShoot();              // GP1 R2 hold
        handleDrivetainAlignment(); // GP1 L2 hold
        handleTurretOdomTracking(); // GP1 L1 toggle
        handleTurretSWMMode();      // GP1 D-pad Up toggle
        handleMisc();               // GP1 R3 + Touchpad + Share
        handleGP1ModeChange();      // GP1 R1 — cycle FAR / CLOSE / AUTO

        handleGP2RPM();               // GP2 R1/L1 fine RPM adjust

        if (shooter.isActive()) {
            shooter.setRPMForDistance(swm.getDistanceForRPM() * 0.0254);
        }

        applyTurretOutput();

        boolean aligned = isTurretAligned()
                && (!drivetrain.isGoalTrackingEnabled() || drivetrain.isAlignedWithGoal());
        lights.update(shooter.isActive(), shooter.getRPMMode(), aligned);

        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        limelight.stop();
        lights.off();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // =========================================================================
    //  DRIVE — always 100% speed, NO drivetrain SWM correction
    // =========================================================================

    private void handleDrive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        // Only apply drivetrain goal tracking correction when enabled and driver not turning.
        // SWM only controls the turret in this opmode — never the drivetrain.
        if (Math.abs(rx) < 0.05 && drivetrain.isGoalTrackingEnabled()) {
            rx = drivetrain.getGoalTrackingTurn();
        }

        follower.setTeleOpDrive(y, -x, -rx, true);
    }

    // =========================================================================
    //  GP1 — SHOOTER TOGGLE (Circle)
    // =========================================================================

    private void handleShooterToggle() {
        if (gamepad1.circle && !lastCircle) {
            shooter.toggle();
            gamepad1.rumble(shooter.isActive() ? 500 : 200);
        }
        lastCircle = gamepad1.circle;
    }

    // =========================================================================
    //  GP1 — INTAKE (Cross / Square / Triangle)
    // =========================================================================

    private void handleIntake() {
        if (gamepad1.cross     && !lastCross)    intake.setMode(Intake.Mode.INTAKE);
        if (gamepad1.square    && !lastSquare)   intake.setMode(Intake.Mode.OFF);
        if (gamepad1.triangle  && !lastTriangle) intake.setMode(Intake.Mode.SPIT);
        lastCross    = gamepad1.cross;
        lastSquare   = gamepad1.square;
        lastTriangle = gamepad1.triangle;
    }

    // =========================================================================
    //  GP1 — SHOOT (R2 hold)
    // =========================================================================

    private void handleShoot() {
        boolean r2 = gamepad1.right_trigger > 0.5;
        if (r2  && !lastR2) intake.setMode(Intake.Mode.SHOOT);
        if (!r2 &&  lastR2) intake.setMode(Intake.Mode.OFF);
        lastR2 = r2;
    }

    // =========================================================================
    //  GP1 — DRIVETRAIN ALIGNMENT (L2 hold → odom align on, release → off)
    // =========================================================================

    private void handleDrivetainAlignment() {
        boolean l2 = gamepad1.left_trigger > 0.5;
        if (l2  && !lastL2) { drivetrain.enableGoalTracking(false); gamepad1.rumble(200); }
        if (!l2 &&  lastL2) drivetrain.disableGoalTracking();
        lastL2 = l2;
    }

    // =========================================================================
    //  GP1 — TURRET ODOM TRACKING (L1 toggle)
    //
    //  First press  → turret tracks goal from current odom pose. Long rumble = ON.
    //  Second press → turret returns to center. Short rumble = OFF.
    // =========================================================================

    private void handleTurretOdomTracking() {
        if (gamepad1.left_bumper && !lastL1) {
            turretOdomTracking = !turretOdomTracking;
            turretLastError    = 0.0;
            turretPidTimer.reset();
            if (turretOdomTracking) {
                gamepad1.rumble(500);
            } else {
                turretServoPos = TURRET_CENTER_POS;
                gamepad1.rumble(200);
            }
        }
        lastL1 = gamepad1.left_bumper;
    }

    // =========================================================================
    //  GP1 — TURRET SWM MODE (D-pad Up toggle — future pose aim)
    // =========================================================================

    private void handleTurretSWMMode() {
        if (gamepad1.dpad_up && !lastDpadUp) {
            turretSWMMode   = !turretSWMMode;
            turretLastError = 0.0;
            turretPidTimer.reset();
            if (turretSWMMode) {
                gamepad1.rumble(700);
            } else {
                turretServoPos = TURRET_CENTER_POS;
                gamepad1.rumble(150);
            }
        }
        lastDpadUp = gamepad1.dpad_up;
    }

    // =========================================================================
    //  GP1 — MISC (R3 reset heading, Touchpad position hold, Share reset odom)
    // =========================================================================

    private void handleMisc() {
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
    }

    // =========================================================================
    //  GP1 — SHOOTER PRESET CYCLE (R1 tap: CLOSE → FAR → AUTO → CLOSE)
    //  Lights automatically reflect the active mode.
    // =========================================================================

    private void handleGP1ModeChange() {
        if (gamepad1.right_bumper && !lastR1) {
            Shooter.RPMMode current = shooter.getRPMMode();
            if      (current == Shooter.RPMMode.CLOSE) { shooter.setFarMode();   gamepad1.rumbleBlips(1); }
            else if (current == Shooter.RPMMode.FAR)   { shooter.setAutoMode();  gamepad1.rumbleBlips(2); }
            else                                        { shooter.setCloseMode(); gamepad1.rumble(200);    }
        }
        lastR1 = gamepad1.right_bumper;
    }

    // =========================================================================
    //  GP2 — RPM FINE ADJUST (R1 = increase, L1 = decrease)
    //
    //  Useful when the driver needs to nudge RPM up or down without cycling
    //  through modes. Works on whatever mode is currently active.
    // =========================================================================

    private void handleGP2RPM() {
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        if (gamepad2.left_bumper  && !lastGP2L1) shooter.decreaseRPM();
        lastGP2R1 = gamepad2.right_bumper;
        lastGP2L1 = gamepad2.left_bumper;
    }

    // =========================================================================
    //  TURRET OUTPUT
    // =========================================================================

    private void applyTurretOutput() {
        if (turretSWMMode) {
            turretServoPos = calculateTurretSWMPosition();
        } else if (turretOdomTracking) {
            turretServoPos = calculateTurretOdomPosition();
        } else {
            turretServoPos  = TURRET_CENTER_POS;
            turretLastError = 0.0;
            turretPidTimer.reset();
        }

        turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
        turret.setManualAngle(servoPositionToAngle(turretServoPos));
        turret.setMode((turretSWMMode || turretOdomTracking)
                ? Turret.Mode.ODOMETRY : Turret.Mode.MANUAL);
    }

    private double calculateTurretSWMPosition() {
        if (!swm.isHeadingLockActive()) return turretServoPos;
        double desiredRad = swm.getTargetHeading() - follower.getPose().getHeading();
        while (desiredRad >  Math.PI) desiredRad -= 2 * Math.PI;
        while (desiredRad < -Math.PI) desiredRad += 2 * Math.PI;
        double deg = Math.toDegrees(desiredRad);
        deg = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, deg));
        return runTurretPD(deg);
    }

    private double calculateTurretOdomPosition() {
        Pose   p     = follower.getPose();
        double angle = Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()))
                - Math.toDegrees(p.getHeading());
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        angle = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
        return runTurretPD(angle);
    }

    private double runTurretPD(double desiredDeg) {
        double current    = servoPositionToAngle(turretServoPos);
        double error      = desiredDeg - current;
        double dt         = turretPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - turretLastError) / dt : 0.0;
        double output     = (error * TURRET_P) + (derivative * TURRET_D);
        turretLastError   = error;
        turretPidTimer.reset();
        return turretServoPos + output;
    }

    private double servoPositionToAngle(double pos) {
        return (pos - TURRET_CENTER_POS) / 0.5 * TURRET_MAX_ANGLE;
    }

    private boolean isTurretAligned() {
        double desired;
        if (turretSWMMode && swm.isHeadingLockActive()) {
            double rel = Math.toDegrees(swm.getTargetHeading() - follower.getPose().getHeading());
            while (rel >  180) rel -= 360;
            while (rel < -180) rel += 360;
            desired = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, rel));
        } else if (turretOdomTracking) {
            Pose   p   = follower.getPose();
            double rel = Math.toDegrees(Math.atan2(GOAL_Y - p.getY(), GOAL_X - p.getX()))
                    - Math.toDegrees(p.getHeading());
            while (rel >  180) rel -= 360;
            while (rel < -180) rel += 360;
            desired = Math.max(-TURRET_MAX_ANGLE, Math.min(TURRET_MAX_ANGLE, rel));
        } else {
            return false;
        }
        return Math.abs(desired - servoPositionToAngle(turretServoPos)) < TURRET_ALIGN_TOLERANCE;
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
        boolean aligned = isTurretAligned()
                && (!drivetrain.isGoalTrackingEnabled() || drivetrain.isAlignedWithGoal());

        String turretModeStr;
        if      (turretSWMMode)      turretModeStr = "SWM FUTURE ✓  [D-up]";
        else if (turretOdomTracking) turretModeStr = "ODOM CURRENT  [L1 toggle]";
        else                         turretModeStr = "CENTER (off)";

        telemetry.addLine("╔═══ SOLO DRIVER TURRET BLUE [TEST] ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "100%  (fixed)");
        telemetry.addData("│ Heading", "%.1f°  (R3 to reset)", headingDeg);

        telemetry.addLine("╠═══ GP1 CONTROLS ═══╣");
        telemetry.addData("│ R1 tap",   "Preset cycle → " + shooter.getRPMMode() + "  (CLOSE→FAR→AUTO)");
        telemetry.addData("│ L2 hold",  "Drivetrain odom align");
        telemetry.addData("│ L1 tap",   "Turret odom toggle → " + (turretOdomTracking ? "ON ✓" : "off"));
        telemetry.addData("│ R2 hold",  "Shoot");
        telemetry.addData("│ Touchpad", "Position hold → " + (drivetrain.isHolding() ? "ON ✓" : "off"));
        telemetry.addData("│ D-up",     "Turret SWM → " + (turretSWMMode ? "ON ✓" : "off"));

        telemetry.addLine("╠═══ GP2 CONTROLS ═══╣");
        telemetry.addData("│ R1 tap",  "RPM up  ↑");
        telemetry.addData("│ L1 tap",  "RPM down ↓");

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X / Y",   "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ TURRET ═══╣");
        telemetry.addData("│ Mode",    turretModeStr);
        telemetry.addData("│ Angle",   "%.1f°  (max ±%.0f°)", currentTurretAngle, TURRET_MAX_ANGLE);
        telemetry.addData("│ Servo",   "%.3f", turretServoPos);
        telemetry.addData("│ Aligned", isTurretAligned() ? "YES ✓" : "NO");

        telemetry.addLine("╠═══ LIGHTS ═══╣");
        telemetry.addData("│ Shooter", shooter.isActive() ? "ON" : "OFF");
        telemetry.addData("│ Mode",    shooter.getRPMMode());
        telemetry.addData("│ State",   aligned ? "BLINK ✓" : "solid");

        telemetry.addLine("╠═══ DRIVETRAIN ALIGN ═══╣");
        telemetry.addData("│ Mode",    drivetrain.isGoalTrackingEnabled() ? "ODOM [L2 hold]" : "OFF");
        telemetry.addData("│ Hold",    drivetrain.isHolding() ? "HOLD ✓ [Touchpad]" : "off");
        telemetry.addData("│ Aligned", drivetrain.isAlignedWithGoal() ? "YES ✓" : "NO");
        telemetry.addData("│ TX",      "%.1f°", limelight.getTx());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Turret Mode", turretSWMMode ? "ON (D-up) ✓" : "OFF");
        telemetry.addData("│ Ready",       swm.isReadyToShoot() ? "SHOOT NOW ✓" : "waiting");
        telemetry.addData("│ Distance",    "%.1f in", swm.getDistanceForRPM());
        telemetry.addData("│ Target Hdg",  "%.1f°", Math.toDegrees(swm.getTargetHeading()));
        telemetry.addData("│ Future X/Y",  "%.1f, %.1f", futurePose.getX(), futurePose.getY());
        telemetry.addData("│ Velocity",    "%.1f in/s", swm.getVelocityMagnitude());

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }
}