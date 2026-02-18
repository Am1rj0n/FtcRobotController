package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

/**
 * Static class to transfer robot pose from Autonomous to TeleOp
 * Allows seamless transition - TeleOp starts where Auto ended
 */
public class AutoToTeleTransfer {
    // Default starting position if auto wasn't run
    public static Pose finalPose = new Pose(72, 8, Math.toRadians(90));
}