package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AutoToTeleTransfer;

@Autonomous(name = "Path Only Blue", group = "Autonomous")
public class AutoTest extends OpMode {

    private Follower follower;
    private ElapsedTime runtime = new ElapsedTime();

    private int pathState = 0;

    private static final int STATE_PATH_1 = 0;
    private static final int STATE_PATH_2 = 1;
    private static final int STATE_PATH_3 = 2;
    private static final int STATE_PATH_4 = 3;
    private static final int STATE_DONE   = 4;

    private PathChain Path1, Path2, Path3, Path4;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        buildPaths();
    }

    @Override
    public void start() {
        runtime.reset();
       //starts
        follower.followPath(Path1);
        pathState = STATE_PATH_1;

        // Save starting pose for teleop
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case STATE_PATH_1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2);
                    pathState = STATE_PATH_2;
                }
                break;

            case STATE_PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    pathState = STATE_PATH_3;
                }
                break;

            case STATE_PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    pathState = STATE_PATH_4;
                }
                break;

            case STATE_PATH_4:
                if (!follower.isBusy()) {
                    pathState = STATE_DONE;
                }
                break;

            case STATE_DONE:
                // Nothing - robot stops here
                break;
        }

        //save pose for teleop transfer
        AutoToTeleTransfer.finalPose = follower.getPose();
        
    }

    @Override
    public void stop() {
        AutoToTeleTransfer.finalPose = follower.getPose();
    }

    private void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(6.014, 8.359)
                ))
                .setTangentHeadingInterpolation()
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(6.014, 8.359),
                        new Pose(21.600, 10.414)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(21.600, 10.414),
                        new Pose(6.662, 10.421)
                ))
                .setTangentHeadingInterpolation()
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(6.662, 10.421),
                        new Pose(56.683, 9.959)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    private void displayTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addLine("╔═══ PATH ONLY AUTO ═══╗");
        telemetry.addData("│ State",   getStateName(pathState));
        telemetry.addData("│ Runtime", "%.1fs", runtime.seconds());
        telemetry.addData("│ Busy",    follower.isBusy() ? "MOVING" : "STOPPED");

        telemetry.addLine("╠═══ POSE ═══╣");
        telemetry.addData("│ X",       "%.2f", pose.getX());
        telemetry.addData("│ Y",       "%.2f", pose.getY());
        telemetry.addData("│ Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("╚═══════════════════════╝");
        telemetry.update();
    }

    private String getStateName(int state) {
        switch (state) {
            case STATE_PATH_1: return "Path 1";
            case STATE_PATH_2: return "Path 2";
            case STATE_PATH_3: return "Path 3";
            case STATE_PATH_4: return "Path 4";
            case STATE_DONE:   return "Done";
            default:           return "Unknown";
        }
    }
}