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

@TeleOp(name = "No Turret Red", group = "Competition")
public class NoTurretRedTeleOp extends OpMode {

    private static final boolean IS_RED        = true;
    private static final double  TURRET_CENTER = 0.5;

    // SWM PD constants
    private static final double SWM_HEADING_P = 2.0;
    private static final double SWM_HEADING_D = 0.05;
    private static final double SWM_MAX_TURN  = 0.6;
    private double              swmLastError  = 0.0;
    private final ElapsedTime   swmPidTimer   = new ElapsedTime();

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU     imu;
    private double  fieldCentricOffset = 0.0;

    private double speedMultiplier              = 0.7;
    private static final double MIN_SPEED       = 0.1;
    private static final double MAX_SPEED       = 1.0;
    private static final double SPEED_INCREMENT = 0.1;

    private Drivetrain            drivetrain;
    private Intake                intake;
    private Shooter               shooter;
    private Limelight             limelight;
    private ShootingWhileMoving   swm;
    private AutoPositionSubsystem autoPos;
    private Lights                lights;
    private Servo                 turretServo;
    private Follower              follower;

    private final ElapsedTime runtime = new ElapsedTime();

    // GP1
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

    // GP2
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
        lights     = new Lights(hardwareMap);
        turretServo = hardwareMap.servo.get("turret");
        turretServo.setPosition(TURRET_CENTER);

        telemetry.addData("Status",    "No Turret Red - Ready");
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

        if (shooter.isActive()) {
            shooter.setRPMForDistance(swm.getDistanceForRPM() * 0.0254);
        }

        turretServo.setPosition(TURRET_CENTER);

        // Lights: aligned = drivetrain aligned (no turret here)
        boolean aligned = drivetrain.isGoalTrackingEnabled() && drivetrain.isAlignedWithGoal();
        lights.update(shooter.isActive(), shooter.getRPMMode(), aligned);

        displayTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        limelight.stop();
        autoPos.cancel();
        lights.off();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void handleGP1Drive() {
        boolean moving = Math.abs(gamepad1.left_stick_x)  > 0.1
                || Math.abs(gamepad1.left_stick_y)  > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1;
        if (moving) { autoPos.cancel(); gateCycleActive = false; }

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.left_bumper  && !lastL1) speedMultiplier = Math.max(MIN_SPEED, speedMultiplier - SPEED_INCREMENT);
        if (gamepad1.right_bumper && !lastR1) speedMultiplier = Math.min(MAX_SPEED, speedMultiplier + SPEED_INCREMENT);
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

        if (gamepad1.share) { drivetrain.resetToCorner(); gamepad1.rumbleBlips(2); }

        boolean driverTurning = Math.abs(rx) > 0.05;
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
        double error      = normalizeAngle(swm.getTargetHeading() - follower.getPose().getHeading());
        double dt         = swmPidTimer.seconds();
        double derivative = (dt > 0.001) ? (error - swmLastError) / dt : 0.0;
        double turn       = Math.max(-SWM_MAX_TURN, Math.min(SWM_MAX_TURN,
                (error * SWM_HEADING_P) + (derivative * SWM_HEADING_D)));
        swmLastError = error;
        swmPidTimer.reset();
        return -turn;
    }

    private double normalizeAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private void handleGP1Alignment() {
        boolean l2 = gamepad1.left_trigger  > 0.5;
        boolean r2 = gamepad1.right_trigger > 0.5;
        if (l2 && !lastL2) { drivetrain.enableGoalTracking(true);  gamepad1.rumble(200); }
        if (r2 && !lastR2) { drivetrain.enableGoalTracking(false); gamepad1.rumble(200); }
        if (!l2 && !r2 && (lastL2 || lastR2)) drivetrain.disableGoalTracking();
        lastL2 = l2; lastR2 = r2;
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
        lastCross = gamepad1.cross; lastSquare = gamepad1.square; lastTriangle = gamepad1.triangle;
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

    private void handleGP2Shoot() {
        boolean r2 = gamepad2.right_trigger > 0.5;
        if (r2  && !lastGP2R2) intake.setMode(Intake.Mode.SHOOT);
        if (!r2 &&  lastGP2R2) intake.setMode(Intake.Mode.OFF);
        lastGP2R2 = r2;
    }

    private void handleGP2Positioning() {
        if (gamepad2.dpad_up && !lastGP2DpadUp) {
            autoPos.goToCloseShoot(); shooter.setCloseMode();
            gateCycleActive = false; gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2DpadDown) {
            autoPos.goToFarShoot(); shooter.setFarMode();
            gateCycleActive = false; gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadDown = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastGP2DpadLeft) {
            autoPos.goToPark(); gateCycleActive = false;
            gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadLeft = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastGP2DpadRight) {
            autoPos.goToGateCycle(); gateCycleActive = true;
            intake.setMode(Intake.Mode.INTAKE); gamepad1.rumble(300); gamepad2.rumble(300);
        }
        lastGP2DpadRight = gamepad2.dpad_right;

        if (gamepad2.cross && !lastGP2X) {
            autoPos.cancel(); gateCycleActive = false; gamepad2.rumbleBlips(2);
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

        if (gamepad2.square && !lastGP2Square) { shooter.setAutoMode();         gamepad2.rumbleBlips(1); }
        if (gamepad2.circle && !lastGP2Circle) { shooter.setManualFromCurrent(); gamepad2.rumbleBlips(1); }
        lastGP2Square = gamepad2.square;
        lastGP2Circle = gamepad2.circle;

        if (gamepad2.left_bumper  && !lastGP2L1) shooter.decreaseRPM();
        if (gamepad2.right_bumper && !lastGP2R1) shooter.increaseRPM();
        lastGP2L1 = gamepad2.left_bumper;
        lastGP2R1 = gamepad2.right_bumper;
    }

    private void handleGP2Localization() {
        if (gamepad2.share && !lastGP2Share) {
            boolean ok = limelight.megaTag2Localize(follower);
            if (ok) { gamepad2.rumble(1000); gamepad1.rumble(500); }
            else    { gamepad2.rumbleBlips(3); }
        }
        lastGP2Share = gamepad2.share;
    }

    private void displayTelemetry() {
        Pose   pose       = follower.getPose();
        double headingDeg = Math.toDegrees(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - fieldCentricOffset);
        boolean aligned = drivetrain.isGoalTrackingEnabled() && drivetrain.isAlignedWithGoal();

        telemetry.addLine("╔═══ NO TURRET RED ═══╗");
        telemetry.addData("│ Intake",  intake.getCurrentMode());
        telemetry.addData("│ Speed",   "%.0f%%", speedMultiplier * 100);
        telemetry.addData("│ Heading", "%.1f°", headingDeg);
        telemetry.addData("│ AutoPos", autoPos.isActive() ? "ACTIVE" : "idle");

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X / Y",   "%.1f, %.1f", pose.getX(), pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╠═══ LIGHTS ═══╣");
        telemetry.addData("│ Shooter", shooter.isActive() ? "ON" : "OFF");
        telemetry.addData("│ Mode",    shooter.getRPMMode());
        telemetry.addData("│ Aligned", aligned ? "YES → BLINK ✓" : "NO → solid");

        telemetry.addLine("╠═══ ALIGNMENT ═══╣");
        telemetry.addData("│ Mode",    drivetrain.isGoalTrackingEnabled()
                ? (drivetrain.isVisionAssistEnabled() ? "LIMELIGHT" : "ODOMETRY") : "OFF");
        telemetry.addData("│ Aligned", drivetrain.isAlignedWithGoal() ? "YES ✓" : "NO");
        telemetry.addData("│ TX",      "%.1f°", limelight.getTx());

        telemetry.addLine("╠═══ SHOOTER ═══╣");
        telemetry.addData("│ %s", shooter.getTelemetryString());

        telemetry.addLine("╠═══ SWM ═══╣");
        telemetry.addData("│ Enabled",  swm.isEnabled() ? "YES" : "NO");
        telemetry.addData("│ Ready",    swm.isReadyToShoot() ? "SHOOT NOW" : "Waiting");
        telemetry.addData("│ Distance", "%.1f in", swm.getDistanceForRPM());

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }
}