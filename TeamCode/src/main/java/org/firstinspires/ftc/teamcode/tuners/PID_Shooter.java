package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ChatGPT Flywheel PID Tuner", group="Tuning")
public class PID_Shooter extends OpMode {

    // motors
    private DcMotorEx fly1, fly2;

    // PID values (starting values YOU requested)
    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;

    // target velocity
    public static double TARGET_RPM = 4000;

    // constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000;

    // RPM measurement
    private double lastPos1 = 0;
    private double lastPos2 = 0;
    private ElapsedTime rpmTimer = new ElapsedTime();

    // PID internal
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;

    // ========== EDGE DETECTION BUTTON STATES ==========
    private boolean prevUp, prevDown, prevLeft, prevRight;
    private boolean prevY, prevA, prevRB, prevLB, prevX;

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        fly2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        fly1.setDirection(DcMotorSimple.Direction.FORWARD);
        fly2.setDirection(DcMotorSimple.Direction.FORWARD);

        fly1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lastPos1 = fly1.getCurrentPosition();
        lastPos2 = fly2.getCurrentPosition();
        rpmTimer.reset();

        telemetry.addLine("=== ChatGPT FLYWHEEL TUNER ===");
        telemetry.addLine("P starts 0.001   (UP squares, DOWN halves)");
        telemetry.addLine("I increases 0.00005 (Y), decreases (A)");
        telemetry.addLine("D increases 0.00001 (RIGHT), decreases (LEFT)");
        telemetry.addLine("RB/LB = RPM target +50/-50");
        telemetry.addLine("X = reset PID");
    }

    @Override
    public void loop() {

        // ================= EDGE DETECTION ==================
        boolean up     = gamepad1.dpad_up     && !prevUp;
        boolean down   = gamepad1.dpad_down   && !prevDown;
        boolean left   = gamepad1.dpad_left   && !prevLeft;
        boolean right  = gamepad1.dpad_right  && !prevRight;
        boolean yPress = gamepad1.y           && !prevY;
        boolean aPress = gamepad1.a           && !prevA;
        boolean rb     = gamepad1.right_bumper && !prevRB;
        boolean lb     = gamepad1.left_bumper  && !prevLB;
        boolean xPress = gamepad1.x            && !prevX;

        // ================= APPLY ONE-CLICK STEPS ==================

        // --- P (square or divide) ---
        if (up)   P = P * P;
        if (down) P = Math.max(0, P / 2);

        // --- I (+/- 0.00005) ---
        if (yPress) I += 0.00005;
        if (aPress) I = Math.max(0, I - 0.00005);

        // --- D (+/- 0.00001) ---
        if (right) D += 0.00001;
        if (left)  D = Math.max(0, D - 0.00001);

        // --- RPM Target ---
        if (rb) TARGET_RPM = Math.min(MAX_RPM, TARGET_RPM + 50);
        if (lb) TARGET_RPM = Math.max(0, TARGET_RPM - 50);

        // --- Reset ---
        if (xPress) {
            lastError = 0;
            integral = 0;
            lastTime = 0;
        }

        // ========== save edge detection states ==========
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevRight = gamepad1.dpad_right;
        prevY = gamepad1.y;
        prevA = gamepad1.a;
        prevRB = gamepad1.right_bumper;
        prevLB = gamepad1.left_bumper;
        prevX = gamepad1.x;

        // ========== MEASURE REAL RPM ==========
        double currentPos1 = fly1.getCurrentPosition();
        double currentPos2 = fly2.getCurrentPosition();

        double dt = rpmTimer.seconds();
        rpmTimer.reset();

        double rpm1 = 0;
        double rpm2 = 0;

        if (dt > 0) {
            rpm1 = ((currentPos1 - lastPos1) / TICKS_PER_REV) * 60.0 / dt;
            rpm2 = ((currentPos2 - lastPos2) / TICKS_PER_REV) * 60.0 / dt;
        }

        lastPos1 = currentPos1;
        lastPos2 = currentPos2;

        double avgRPM = (rpm1 + rpm2) / 2.0;

        // ========== PID CONTROLLER ==========
        double error = TARGET_RPM - avgRPM;

        double t = getRuntime();
        double dtPID = (lastTime == 0) ? 0 : (t - lastTime);

        double derivative = 0;

        if (dtPID > 0) {
            integral += error * dtPID;
            derivative = (error - lastError) / dtPID;
        }

        double pidOut = P * error + I * integral + D * derivative;

        lastError = error;
        lastTime = t;

        // ========== APPLY POWER ==========
        double ff = TARGET_RPM / MAX_RPM;  // feedforward
        double power = ff + pidOut / 6000.0;

        power = Math.max(0, Math.min(1, power));

        fly1.setPower(power);
        fly2.setPower(power);

        // ========== TELEMETRY ==========
        telemetry.addLine("=== PID ===");
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addLine();

        telemetry.addLine("=== RPM ===");
        telemetry.addData("Target", TARGET_RPM);
        telemetry.addData("Motor 1", rpm1);
        telemetry.addData("Motor 2", rpm2);
        telemetry.addData("AVG", avgRPM);
        telemetry.addLine();

        telemetry.addData("Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        fly1.setPower(0);
        fly2.setPower(0);
    }
}
