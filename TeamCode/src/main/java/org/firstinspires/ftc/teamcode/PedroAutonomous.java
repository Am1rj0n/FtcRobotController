package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Autonomous Blue", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Subsystems
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    private Limelight limelight;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // Constants
    private static final boolean IS_RED = false;
    private static final double SHOOT_DURATION = 2.0; // seconds

    // State machine states
    private static final int STATE_INIT_SHOOT = 0;
    private static final int STATE_SHOOTING_START = 1;
    private static final int STATE_PATH_1 = 2;
    private static final int STATE_PATH_2 = 3;
    private static final int STATE_PATH_3 = 4;
    private static final int STATE_PATH_4 = 5;
    private static final int STATE_FINAL_SHOOT = 6;
    private static final int STATE_SHOOTING_END = 7;
    private static final int STATE_DONE = 8;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro with predictive braking enabled
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        // Initialize subsystems
        limelight = new Limelight(hardwareMap, IS_RED);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap, limelight, IS_RED);

        // Build paths
        paths = new Paths(follower);

        // Initial state
        pathState = STATE_INIT_SHOOT;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", IS_RED ? "RED" : "BLUE");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        limelight.start();
        runtime.reset();

        // Save starting pose for teleop
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();

        // Update subsystems
        shooter.periodic();
        turret.update(follower.getPose());

        // State machine
        pathState = autonomousPathUpdate();

        // Save pose for teleop transfer
        AutoToTeleTransfer.finalPose = follower.getPose();

        // Telemetry
        panelsTelemetry.debug("State", getStateName(pathState));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooter", shooter.isActive() ? "ON" : "OFF");
        panelsTelemetry.debug("Intake", intake.getCurrentMode().toString());
        panelsTelemetry.debug("Turret Aligned", turret.isAligned() ? "YES" : "NO");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Save final pose for teleop
        AutoToTeleTransfer.finalPose = follower.getPose();

        shooter.stop();
        intake.stop();
        limelight.stop();
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_INIT_SHOOT:
                // Prepare for first shot
                shooter.spin();
                turret.setMode(Turret.Mode.ODOMETRY); // Track goal using odometry

                // Set RPM based on distance
                double distanceMeters = turret.distanceToGoalMeters(follower.getPose());
                shooter.setRPMForDistance(distanceMeters);

                // Wait for shooter to spin up and turret to align
                if (shooter.isAtSpeed() && turret.isAligned()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    return STATE_SHOOTING_START;
                }
                return STATE_INIT_SHOOT;

            case STATE_SHOOTING_START:
                // Shooting for 2 seconds
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.stop();

                    // Start first path
                    follower.followPath(paths.Path1);
                    return STATE_PATH_1;
                }
                return STATE_SHOOTING_START;

            case STATE_PATH_1:
                // Following Path 1
                if (!follower.isBusy()) {
                    // Start Path 2 with intake
                    intake.setMode(Intake.Mode.INTAKE);
                    follower.followPath(paths.Path2);
                    return STATE_PATH_2;
                }
                return STATE_PATH_1;

            case STATE_PATH_2:
                // Following Path 2 with intake ON
                if (!follower.isBusy()) {
                    // Continue to Path 3, keep intake on
                    follower.followPath(paths.Path3);
                    return STATE_PATH_3;
                }
                return STATE_PATH_2;

            case STATE_PATH_3:
                // Following Path 3 with intake still ON
                if (!follower.isBusy()) {
                    // Turn off intake, start Path 4
                    intake.setMode(Intake.Mode.OFF);
                    follower.followPath(paths.Path4);
                    return STATE_PATH_4;
                }
                return STATE_PATH_3;

            case STATE_PATH_4:
                // Following Path 4
                if (!follower.isBusy()) {
                    // Prepare for final shot
                    shooter.spin();
                    turret.setMode(Turret.Mode.ODOMETRY);

                    double finalDistance = turret.distanceToGoalMeters(follower.getPose());
                    shooter.setRPMForDistance(finalDistance);

                    return STATE_FINAL_SHOOT;
                }
                return STATE_PATH_4;

            case STATE_FINAL_SHOOT:
                // Wait for alignment and speed
                if (shooter.isAtSpeed() && turret.isAligned()) {
                    shootTimer.reset();
                    intake.setMode(Intake.Mode.SHOOT);
                    return STATE_SHOOTING_END;
                }
                return STATE_FINAL_SHOOT;

            case STATE_SHOOTING_END:
                // Final shooting for 2 seconds
                if (shootTimer.seconds() >= SHOOT_DURATION) {
                    intake.setMode(Intake.Mode.OFF);
                    shooter.stop();
                    return STATE_DONE;
                }
                return STATE_SHOOTING_END;

            case STATE_DONE:
                // Autonomous complete
                return STATE_DONE;

            default:
                return STATE_DONE;
        }
    }

    private String getStateName(int state) {
        switch (state) {
            case STATE_INIT_SHOOT: return "Init Shoot";
            case STATE_SHOOTING_START: return "Shooting Start";
            case STATE_PATH_1: return "Path 1";
            case STATE_PATH_2: return "Path 2 (Intake)";
            case STATE_PATH_3: return "Path 3 (Intake)";
            case STATE_PATH_4: return "Path 4";
            case STATE_FINAL_SHOOT: return "Final Shoot";
            case STATE_SHOOTING_END: return "Shooting End";
            case STATE_DONE: return "Done";
            default: return "Unknown";
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            // Predictive braking automatically applied from Constants
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(6.014, 8.359)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.014, 8.359),
                                    new Pose(21.600, 10.414)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.600, 10.414),
                                    new Pose(6.662, 10.421)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.662, 10.421),
                                    new Pose(56.683, 9.959)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}

/**
 * Static class to transfer pose from Auto to TeleOp
 */
class AutoToTeleTransfer {
    public static Pose finalPose = new Pose(72, 8, Math.toRadians(90)); // Default if auto not run
}