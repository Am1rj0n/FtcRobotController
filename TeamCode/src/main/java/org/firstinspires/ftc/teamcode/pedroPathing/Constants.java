package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {


    // ========== SHOOTER PIDF COEFFICIENTS ==========
    // These control how the shooter maintains target velocity
    // Tuned for velocity in degrees/second (not RPM!)
    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(
            0.005,      // P: Proportional gain - increase if too slow, decrease if oscillates
            0.0002,     // I: Integral gain - helps eliminate steady-state error
            0.0001,     // D: Derivative gain - dampens oscillations
            0.0         // F: Feedforward - leave at 0 for now
    );

    // ========== SHOOTER PRESET VELOCITIES ==========
    // All velocities in degrees/second

    //36000 = 6000RPM
    public static final double closeShootPower = 16000;   // Close range shooting (CHange to 2000-3000 if not work)
    public static final double farShootPower = 28000;     // Far range shooting
    public static final double idleShootPower = 5000;     // Idle speed (0.2 power equivalent = ~360 RPM = 72 deg/s at 20% of 1800 RPM)

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.9)
        .forwardZeroPowerAcceleration(-29.1309918467)
            .lateralZeroPowerAcceleration(-77.65507688407813)


           .translationalPIDFCoefficients(new PIDFCoefficients(
                   0.15,
                    0.0001,
                    0.02,
                    0.12
            ))
            .translationalPIDFSwitch(3)
            .headingPIDFCoefficients(new PIDFCoefficients(
                       2.0,
                      0.005,
                     0.2,
                     0.03
             ))
            .headingPIDFSwitch(0.1)
             .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                     0.15,
                     0,
                     0.0005,
                     0.7,
                     0.02
             ))
            .drivePIDFSwitch(8)
            .centripetalScaling(0.0003);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("front_left_motor")
            .leftRearMotorName("back_left_motor")
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)



            .xVelocity(62.46352837029404)
            .yVelocity(49.594572773129926);

    public static PinpointConstants localizerConstants = new PinpointConstants()

            .forwardPodY(-2.125)
            .strafePodX(-2.75)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            1.0,
            1.0,
            0.030,
            50,
            2.0,
            15,
            2
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
