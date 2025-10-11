package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // TODO: After tuning, update this with your robot's zero power deceleration going forward
    public static FollowerConstants followerConstants = new FollowerConstants()

            //I USED DUA PID (1 MAIN, 2 FOR SMALL CORRECTIONS)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)


            .forwardZeroPowerAcceleration(30)  // placeholder value
            .lateralZeroPowerAcceleration(30); // placeholder value





    // TODO: After tuning, update these max velocities (in inches per second)
    public static PathConstraints pathConstraints = new PathConstraints(
            50,  // max velocity - TODO: replace with Forward Velocity Tuner result
            100, // max acceleration - you can tune this later if needed
            1,   // max angular velocity (radians/sec)
            1    // max angular acceleration (radians/sec^2)
    );

    // TODO: After tuning, update these velocities (in inches per second)
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(50)  // TODO: replace with Forward Velocity Tuner result
            .yVelocity(50)  // TODO: replace with Lateral Velocity Tuner result
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftRearMotorName("back_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
