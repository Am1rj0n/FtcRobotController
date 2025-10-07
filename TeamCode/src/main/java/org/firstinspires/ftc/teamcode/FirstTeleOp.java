
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;








@TeleOp(name = "MecanumTeleOP", group = "TeleOp")
public class FirstTeleOp extends OpMode {




    // Code goes here:
    private DcMotor frontleft, frontright; //variables go here. I set variables for motors
    private DcMotor backleft, backright;




    @Override
    public void init() {
        //init runs once before match starts
        //hardwareMap connects to the actual motor on the control/expansion hub
        frontleft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontright = hardwareMap.get(DcMotor.class, "front_right_motor");
        backleft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backright = hardwareMap.get(DcMotor.class, "back_right_motor");



    }




    public void loop() { //


        double driverspeed = 1.0; //Adjust for speed, 1=100%. LESS = Precision
        double max; //temporary holder, used to balance wheel power
















        //Read Joysticks : Set Controls
        //axial: forward/back
        //lateral: strafing
        //yaw: Rotation


        // Each joystick axis goes from -1 to 1
        //So push full forward = -1, pull back = +1


//REMEMBER Y STICK IS INVERTED
// We add a - sign because the stick is backwards (pushing up gives negative numbers).
//Also set gwhat gamepad does
        double axial = -gamepad1.left_stick_y;   // forward/back (Left joystick y axis)
        double lateral = gamepad1.left_stick_x;   // strafe left/right (Left joystick x axis
        double yaw = gamepad1.right_stick_x;       // rotation (Right joystick x axis)














        //Mecanum Wheel Power Formula
        //Lets robot drive holonomically (All directional drive)
        //It mixes forward/back, strafe, and rotation into power for each wheel
        double leftFrontPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = axial + lateral - yaw;


//example: Push up stick all the way up and to the side to make 45 degrees. Think of a triangle
// map out like UNIT CIRCLE to understand. OR use my joystick mapper on github
//Y stick left is -.71. X stick left is .71 and right stick x is lets say 0. (No rotation)
// Axial = -(-.71) = +.71  Lateral= +.71. yaw=0 = .71 - .71 +0 which = 0 so front left motor dont move but
// Axial = -(-.71) Lateral = +.71 yaw =0 so .71+.71 +0 which is 1.42 so front back motor moves
// this makes sense because the this joystick position is strafing top right and front motor shouldnt move based on the mecanum wheels








        //finds the biggest wheel power value.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));






        //If any wheel tries to go above 1.0 power, this scales them all down evenly
        //That way, no motor gets an invalid number (motors only accept -1.0 to 1.0)
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }










        //Set Motor Powers
        frontleft.setPower(leftFrontPower * driverspeed);
        frontright.setPower(rightFrontPower * driverspeed);
        backleft.setPower(leftBackPower * driverspeed);
        backright.setPower(rightBackPower * driverspeed);
//Example: if leftFrontPower = 0.9 and driverspeed = 0.7 → motor actually runs at 0.63.












//Source for this: https://gm0.org/en/latest/docs/software/tutorials/using-telemetry.html#updating-telemetry




        //This prints data on the Driver Hub screen while you drive.
        //You can see joystick values and motor powers in real time.
        //telemetry.update() tells the robot to refresh the screen each loop.




        // Debugging Telemetry
        telemetry.addData("Axial (Forward/Back)", axial);
        telemetry.addData("Lateral (Strafe)", lateral);
        telemetry.addData("Yaw (Rotate)", yaw);


        telemetry.addData("Front Left Power", leftFrontPower);
        telemetry.addData("Front Right Power", rightFrontPower);
        telemetry.addData("Back Left Power", leftBackPower);
        telemetry.addData("Back Right Power", rightBackPower);


        telemetry.update(); //must end like this




    }
}
