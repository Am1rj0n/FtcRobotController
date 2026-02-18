package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class constantsencoder  {

    //tune these
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.9) //kg
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
            .leftFrontMotorName("lf")
            .leftRearMotorName("lr")
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)


            //tune
            .xVelocity(62.46352837029404)
            .yVelocity(49.594572773129926);

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)

            .robotWidth(16)
            .robotLength(18);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            0.90,
            1.0,
            0.030,
            100,
            1.0,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {

        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
}
