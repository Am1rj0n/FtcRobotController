package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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

    private static final PredictiveBrakingCoefficients BRAKING_COEFFICIENTS =
            new PredictiveBrakingCoefficients(
                    0.5,
                    0.05,
                    0.001
            ).withMaximumBrakingPower(0.8);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.431)
        .forwardZeroPowerAcceleration(-46.0668)
            .lateralZeroPowerAcceleration(-72.720259)


           .translationalPIDFCoefficients(new PIDFCoefficients(
                   0.14,
                    0.0001,
                    0.02,
                    0.05
            ))
            .translationalPIDFSwitch(3)
            .headingPIDFCoefficients(new PIDFCoefficients(
                       2.2,
                      0.005,
                     0.2,
                     0.01
             ))
            .headingPIDFSwitch(0.1)

            //tune drive

            //increase P until oscillatin, then increase D to get rid = agressive movement
             .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                     0.15,
                     0,
                     0.0007,
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



            .xVelocity(70.266922)
            .yVelocity(53.334577);

    public static PinpointConstants localizerConstants = new PinpointConstants()

            .forwardPodY(-0.245)
            .strafePodX(-3.697)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            1.0,
            0.95,
            1.0,
            0.030,
            50,
            1.67,
            10,
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
