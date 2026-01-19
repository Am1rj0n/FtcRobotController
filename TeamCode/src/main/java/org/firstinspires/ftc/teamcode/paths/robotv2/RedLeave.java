package org.firstinspires.ftc.teamcode.paths.robotv2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subfilesV2.LightingSubsystem;

@Autonomous(name = "Leave Auto Red Far", group = "Autonomous")
@Configurable
public class RedLeave extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private LightingSubsystem lights;

    // Shooter Hardware
    private DcMotor shooterTop, shooterBottom;

    public static Pose autoEndPose = null;

    private enum AutoState {
        LEAVE, IDLE
    }
    private AutoState currentState = AutoState.LEAVE;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Mirror the blue starting pose for red alliance
        Pose bluePose = new Pose(56, 8, Math.toRadians(90));
        follower.setStartingPose(bluePose.mirror());

        paths = new Paths(follower);

        // Initialize shooter motors to ensure they're off
        shooterTop = hardwareMap.get(DcMotor.class, "outtakeMotor2");
        shooterBottom = hardwareMap.get(DcMotor.class, "outtakeMotor1");

        shooterTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lights = new LightingSubsystem(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        currentState = AutoState.LEAVE;
        follower.followPath(paths.Leave);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        lights.update();

        // Ensure shooter motors are off
        shooterTop.setPower(0);
        shooterBottom.setPower(0);

        panelsTelemetry.debug("State", currentState.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case LEAVE:
                if (!follower.isBusy()) {
                    currentState = AutoState.IDLE;
                }
                break;

            case IDLE:
                // Do nothing, autonomous complete
                break;
        }
    }

    @Override
    public void stop() {
        shooterTop.setPower(0);
        shooterBottom.setPower(0);
        autoEndPose = follower.getPose();
    }

    public static class Paths {
        public PathChain Leave;

        public Paths(Follower follower) {
            // Define blue alliance positions then mirror for red
            Pose blueStart = new Pose(56, 8);
            Pose blueLeave = new Pose(50.393, 22.331);

            // Mirror the path for red alliance
            Leave = follower.pathBuilder().addPath(
                    new BezierLine(
                            blueStart.mirror(),
                            blueLeave.mirror()
                    )
            ).setTangentHeadingInterpolation().build();
        }
    }
}