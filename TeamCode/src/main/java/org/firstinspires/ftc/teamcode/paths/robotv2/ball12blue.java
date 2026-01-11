
package org.firstinspires.ftc.teamcode.paths.robotv2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DualMotorShooterHelper;

@Autonomous(name = "12 Ball Blue New", group = "Autonomous")
@Configurable
public class ball12blue extends OpMode {

    // PUBLIC STATIC POSE FOR TELEOP
    public static Pose autoEndPose = new Pose(0, 0, 0);

    /* ================= PEDRO PATHING ================= */
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    /* ================= INTAKE SYSTEM ================= */
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DistanceSensor intakeSensor;

    private ElapsedTime jamTimer = new ElapsedTime();
    private boolean objectDetected = false;
    private boolean jammed = false;

    private static final double JAM_DISTANCE_CM = 10.0;
    private static final double JAM_TIME = 0.2; // 200ms

    /* ================= SHOOTER SYSTEM ================= */
    private DualMotorShooterHelper shooter;
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_DURATION = 2.0; // 2000ms

    /* ================= STATE MACHINE ================= */
    private enum AutoState {
        PRELOAD_SHOOT,      // State 0: Shoot preload
        INTAKE_1,           // State 1: Drive to intake sample 1
        SHOOT_1,            // State 2: Return and shoot sample 1
        GATE_INTAKE,        // State 3: Go to gate and intake
        SHOOT_2,            // State 4: Shoot sample from gate
        INTAKE_2,           // State 5: Intake another sample
        SHOOT_3,            // State 6: Final shoot
        LEAVE,              // State 7: Park
        IDLE                // State 8: Done
    }

    private AutoState currentState = AutoState.PRELOAD_SHOOT;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.779, 109.959, Math.toRadians(90)));
        paths = new Paths(follower);

        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        shooter = new DualMotorShooterHelper(
                hardwareMap,
                Constants.shooterCoefficients
        );

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized - 12 Ball Blue");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start with shooter warmup for preload
        shooter.runCloseShot();
        shootTimer.reset();
    }

    @Override
    public void loop() {
        // Update systems
        follower.update();
        shooter.update();
        updateJamDetection();

        // Run state machine
        autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Jammed", jammed);
        panelsTelemetry.debug("Shooter Ready", shooter.isAtTargetVelocity());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);


    }

    @Override
    public void stop() {
        // Save final pose for TeleOp
        autoEndPose = follower.getPose();

        // Stop all systems
        stopIntake();
        shooter.stop();
    }

    /* ================= JAM DETECTION ================= */
    private void updateJamDetection() {
        double d = intakeSensor.getDistance(DistanceUnit.CM);

        if (d < JAM_DISTANCE_CM) {
            if (!objectDetected) {
                objectDetected = true;
                jamTimer.reset();
            } else if (jamTimer.seconds() > JAM_TIME) {
                jammed = true;
            }
        } else {
            objectDetected = false;
            jammed = false;
            jamTimer.reset();
        }
    }

    /* ================= STATE MACHINE ================= */
    public void autonomousPathUpdate() {
        switch (currentState) {

            case PRELOAD_SHOOT:
                // Shoot preloaded sample
                runShootMode();

                if (shootTimer.seconds() > SHOOT_DURATION) {
                    shooter.stop();
                    follower.followPath(paths.shoot0);
                    currentState = AutoState.INTAKE_1;
                }
                break;

            case INTAKE_1:
                // Drive to first sample and intake
                runIntakeMode();

                if (!follower.isBusy()) {
                    follower.followPath(paths.intake1);
                    currentState = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                // Return to shooting position
                stopIntake();
                shooter.runCloseShot(); // Warm up shooter

                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    shootTimer.reset();
                    currentState = AutoState.GATE_INTAKE;
                }
                break;

            case GATE_INTAKE:
                // Shoot then go to gate - JAM PROTECTION DISABLED
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    shooter.stop();

                    if (!follower.isBusy()) {
                        follower.followPath(paths.gateintake);
                        // Override jam protection for gate intake
                        intakeMotor.setPower(-1.0);
                        transferMotor.setPower(-0.15);
                        currentState = AutoState.SHOOT_2;
                    }
                }
                break;

            case SHOOT_2:
                // Continue intake until path done, then shoot
                if (!follower.isBusy()) {
                    stopIntake();
                    shooter.runCloseShot();
                    follower.followPath(paths.shoot2);
                    shootTimer.reset();
                    currentState = AutoState.INTAKE_2;
                }
                break;

            case INTAKE_2:
                // Shoot then intake another sample
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    shooter.stop();

                    if (!follower.isBusy()) {
                        follower.followPath(paths.intake);
                        runIntakeMode();
                        currentState = AutoState.SHOOT_3;
                    }
                }
                break;

            case SHOOT_3:
                // Final shoot sequence
                stopIntake();
                shooter.runCloseShot();

                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot3);
                    shootTimer.reset();
                    currentState = AutoState.LEAVE;
                }
                break;

            case LEAVE:
                // Shoot then park
                if (shootTimer.seconds() < SHOOT_DURATION) {
                    runShootMode();
                } else {
                    shooter.stop();

                    if (!follower.isBusy()) {
                        follower.followPath(paths.leave);
                        currentState = AutoState.IDLE;
                    }
                }
                break;

            case IDLE:
                // Stop everything
                stopIntake();
                shooter.stop();
                break;
        }

        pathState = currentState.ordinal();
    }

    /* ================= INTAKE CONTROL ================= */
    private void runIntakeMode() {
        if (jammed) {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        } else {
            intakeMotor.setPower(-1.0);
            transferMotor.setPower(-0.15);
        }
    }

    private void stopIntake() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    /* ================= SHOOT CONTROL ================= */
    private void runShootMode() {
        shooter.runCloseShot();
        intakeMotor.setPower(-0.55);
        transferMotor.setPower(0.55);
    }

    /* ================= PATH DEFINITIONS ================= */
    public static class Paths {
        public PathChain shoot0;
        public PathChain intake1;
        public PathChain shoot1;
        public PathChain gateintake;
        public PathChain shoot2;
        public PathChain intake;
        public PathChain shoot3;
        public PathChain leave;

        public Paths(Follower follower) {
            shoot0 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.779, 109.959),
                                    new Pose(57.655, 78.207)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(129))
                    .setReversed()
                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.655, 78.207),
                                    new Pose(56.858, 59.569),
                                    new Pose(22.441, 59.469)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.441, 59.469),
                                    new Pose(58.497, 77.745)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(129))
                    .build();

            gateintake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.497, 77.745),
                                    new Pose(11.103, 59.469)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(327))
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(11.103, 59.469),
                                    new Pose(73.931, 57.838),
                                    new Pose(58.262, 77.876)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.262, 77.876),
                                    new Pose(38.024, 82.462),
                                    new Pose(20.379, 83.738)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.379, 83.738),
                                    new Pose(40.410, 99.810),
                                    new Pose(58.124, 77.869)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(176), Math.toRadians(129))
                    .setReversed()
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.124, 77.869),
                                    new Pose(50.814, 72.738)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(129))
                    .build();
        }
    }
}