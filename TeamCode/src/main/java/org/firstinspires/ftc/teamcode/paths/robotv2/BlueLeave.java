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

@Autonomous(name = "Leave Auto Blue Far", group = "Autonomous")
@Configurable
public class BlueLeave extends OpMode {

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
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

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
            // Simple path from starting position to park position
            // Matches the "Leave" path from Ball12BlueFar
            Leave = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56, 8, Math.toRadians(90)),
                            new Pose(50.393, 22.331)
                    )
            ).setTangentHeadingInterpolation().build();
        }
    }
}