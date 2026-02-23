package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class AutoPositionSubsystem {

    private final Follower follower;
    private final boolean  isRed;
    private boolean isActive = false;

    private static final Pose BLUE_PARK        = new Pose(105.103, 32.938, Math.toRadians(90));
    private static final Pose BLUE_CLOSE_SHOOT = new Pose(57.766,  86.731, Math.toRadians(134));
    private static final Pose BLUE_FAR_SHOOT   = new Pose(58.579,  13.828, Math.toRadians(113.5));

    private static final Pose RED_PARK        = BLUE_PARK.mirror();
    private static final Pose RED_CLOSE_SHOOT = BLUE_CLOSE_SHOOT.mirror();
    private static final Pose RED_FAR_SHOOT   = BLUE_FAR_SHOOT.mirror();

    public AutoPositionSubsystem(Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed    = isRed;
    }

    public void goToCloseShoot() {
        Pose target = isRed ? RED_CLOSE_SHOOT : BLUE_CLOSE_SHOOT;
        follower.followPath(buildPath(target));
        isActive = true;
    }

    public void goToFarShoot() {
        Pose target = isRed ? RED_FAR_SHOOT : BLUE_FAR_SHOOT;
        follower.followPath(buildPath(target));
        isActive = true;
    }

    public void goToPark() {
        Pose target = isRed ? RED_PARK : BLUE_PARK;
        follower.followPath(buildPath(target));
        isActive = true;
    }

    public void cancel() {
        if (isActive) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            isActive = false;
        }
    }

    public void update() {
        if (isActive && !follower.isBusy()) {
            isActive = false;
        }
    }

    private PathChain buildPath(Pose target) {
        return follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), target)))
                .setConstantHeadingInterpolation(target.getHeading())
                .build();
    }

    public boolean isActive() { return isActive; }
}