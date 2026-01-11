package org.firstinspires.ftc.teamcode.subfilesV2;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class AutoPositionSubsystem {

    private final Follower follower;
    private final boolean isRed;

    private boolean isActive = false;

    // Blue alliance positions
    private static final Pose BLUE_CLOSE_SHOOT = new Pose(57.655, 78.207, Math.toRadians(129));
    private static final Pose BLUE_FAR_SHOOT = new Pose(61.297, 16.441, Math.toRadians(112));
    private static final Pose BLUE_PARK = new Pose(105.324, 33.159, Math.toRadians(90));

    // Red alliance positions (mirrored)
    private static final Pose RED_CLOSE_SHOOT = BLUE_CLOSE_SHOOT.mirror();
    private static final Pose RED_FAR_SHOOT = BLUE_FAR_SHOOT.mirror();
    private static final Pose RED_PARK = BLUE_PARK.mirror();

    private static final double AUTO_DRIVE_POWER = 0.85;

    public AutoPositionSubsystem(Follower follower, boolean isRed) {
        this.follower = follower;
        this.isRed = isRed;
    }

    public void goToCloseShoot() {
        Pose target = isRed ? RED_CLOSE_SHOOT : BLUE_CLOSE_SHOOT;
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), target)))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        target.getHeading()
                )
                .build();

        follower.followPath(path);
        follower.setMaxPower(AUTO_DRIVE_POWER);
        isActive = true;
    }

    public void goToFarShoot() {
        Pose target = isRed ? RED_FAR_SHOOT : BLUE_FAR_SHOOT;
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), target)))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        target.getHeading()
                )
                .build();

        follower.followPath(path);
        follower.setMaxPower(AUTO_DRIVE_POWER);
        isActive = true;
    }

    public void goToPark() {
        Pose target = isRed ? RED_PARK : BLUE_PARK;
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower.getPose(), target)))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(),
                        target.getHeading()
                )
                .build();

        follower.followPath(path);
        follower.setMaxPower(AUTO_DRIVE_POWER);
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
        // Auto-cancel when path is complete
        if (isActive && !follower.isBusy()) {
            isActive = false;
        }
    }

    public boolean isActive() {
        return isActive;
    }
}