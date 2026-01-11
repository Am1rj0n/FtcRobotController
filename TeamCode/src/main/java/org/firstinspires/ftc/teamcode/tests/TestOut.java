package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "OuttakeTest (Individual)", group = "Testing")
public class TestOut extends LinearOpMode {

    // Outtake motors
    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;

    // Outtake state - SEPARATE for each motor
    private double outtakePower1 = 0.3;   // Motor 1: 0.0–1.0
    private double outtakePower2 = 0.3;   // Motor 2: 0.0–1.0
    private boolean outtake1Enabled = false;
    private boolean outtake2Enabled = false;

    // Button state
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // PID constants (tune these)
    private static final double P = 0.003;
    private static final double I = 0.0001;
    private static final double D = 0.0;

    // Motor specs
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO = 1.0;
    private static final double MAX_RPM = 6000.0;

    // Velocity tracking
    private double lastPos1 = 0;
    private double lastPos2 = 0;
    private final ElapsedTime velocityTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Hardware mapping
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        // Motor direction
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Encoder setup
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake mode
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init PID - SEPARATE controllers for each motor
        ShooterPIDController pidController1 = new ShooterPIDController();
        ShooterPIDController pidController2 = new ShooterPIDController();
        pidController1.setPID(P, I, D);
        pidController2.setPID(P, I, D);

        lastPos1 = outtakeMotor1.getCurrentPosition();
        lastPos2 = outtakeMotor2.getCurrentPosition();
        velocityTimer.reset();

        // Init telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("MOTOR 1:");
        telemetry.addLine("  Right Trigger: Enable");
        telemetry.addLine("  Left Trigger: Disable");
        telemetry.addLine("  Dpad Up: Increase Power");
        telemetry.addLine("  Dpad Down: Decrease Power");
        telemetry.addLine("MOTOR 2:");
        telemetry.addLine("  Right Bumper: Enable");
        telemetry.addLine("  Left Bumper: Disable");
        telemetry.addLine("  Dpad Right: Increase Power");
        telemetry.addLine("  Dpad Left: Decrease Power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === MOTOR 1 POWER ADJUST ===
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !lastDpadUp) {
                outtakePower1 = Math.min(outtakePower1 + 0.02, 1.0);
            }
            if (dpadDown && !lastDpadDown) {
                outtakePower1 = Math.max(outtakePower1 - 0.02, 0.0);
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // === MOTOR 2 POWER ADJUST ===
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;

            if (dpadRight && !lastDpadRight) {
                outtakePower2 = Math.min(outtakePower2 + 0.02, 1.0);
            }
            if (dpadLeft && !lastDpadLeft) {
                outtakePower2 = Math.max(outtakePower2 - 0.02, 0.0);
            }

            lastDpadRight = dpadRight;
            lastDpadLeft = dpadLeft;

            // === MOTOR 1 ENABLE / DISABLE ===
            if (gamepad1.left_trigger > 0.5) {
                outtake1Enabled = false;
                pidController1.reset();
            }
            if (gamepad1.right_trigger > 0.5) {
                outtake1Enabled = true;
            }

            // === MOTOR 2 ENABLE / DISABLE ===
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (lb && !lastLeftBumper) {
                outtake2Enabled = false;
                pidController2.reset();
            }
            if (rb && !lastRightBumper) {
                outtake2Enabled = true;
            }

            lastLeftBumper = lb;
            lastRightBumper = rb;

            // === MOTOR 1 CONTROL ===
            if (outtake1Enabled) {
                double targetRPM1 = outtakePower1 * MAX_RPM;
                double rpm1 = getMotorVelocityRPM(outtakeMotor1, true);
                double pidOutput1 = pidController1.calculate(rpm1, targetRPM1);
                double feedforward1 = targetRPM1 / MAX_RPM;
                double finalPower1 = feedforward1 + (pidOutput1 / MAX_RPM);
                finalPower1 = Math.max(0.0, Math.min(1.0, finalPower1));

                outtakeMotor1.setPower(finalPower1);

                telemetry.addLine("=== MOTOR 1 ===");
                telemetry.addData("Enabled", "YES");
                telemetry.addData("Power Setting", "%.0f%%", outtakePower1 * 100);
                telemetry.addData("Target RPM", "%.0f", targetRPM1);
                telemetry.addData("Current RPM", "%.0f", rpm1);
                telemetry.addData("RPM Error", "%.0f", targetRPM1 - rpm1);
                telemetry.addData("Final Power", "%.2f", finalPower1);

            } else {
                outtakeMotor1.setPower(0);
                telemetry.addLine("=== MOTOR 1 ===");
                telemetry.addData("Enabled", "NO");
                telemetry.addData("Power Setting", "%.0f%%", outtakePower1 * 100);
            }

            // === MOTOR 2 CONTROL ===
            if (outtake2Enabled) {
                double targetRPM2 = outtakePower2 * MAX_RPM;
                double rpm2 = getMotorVelocityRPM(outtakeMotor2, false);
                double pidOutput2 = pidController2.calculate(rpm2, targetRPM2);
                double feedforward2 = targetRPM2 / MAX_RPM;
                double finalPower2 = feedforward2 + (pidOutput2 / MAX_RPM);
                finalPower2 = Math.max(0.0, Math.min(1.0, finalPower2));

                outtakeMotor2.setPower(finalPower2);

                telemetry.addLine("=== MOTOR 2 ===");
                telemetry.addData("Enabled", "YES");
                telemetry.addData("Power Setting", "%.0f%%", outtakePower2 * 100);
                telemetry.addData("Target RPM", "%.0f", targetRPM2);
                telemetry.addData("Current RPM", "%.0f", rpm2);
                telemetry.addData("RPM Error", "%.0f", targetRPM2 - rpm2);
                telemetry.addData("Final Power", "%.2f", finalPower2);

            } else {
                outtakeMotor2.setPower(0);
                telemetry.addLine("=== MOTOR 2 ===");
                telemetry.addData("Enabled", "NO");
                telemetry.addData("Power Setting", "%.0f%%", outtakePower2 * 100);
            }

            telemetry.update();
        }

        // Stop motors
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    // === RPM CALCULATION ===
    private double getMotorVelocityRPM(DcMotorEx motor, boolean isMotor1) {
        double currentPos = motor.getCurrentPosition();
        double elapsed = velocityTimer.seconds();

        if (elapsed < 0.01) return 0;

        double deltaTicks = currentPos - (isMotor1 ? lastPos1 : lastPos2);
        double rpm = (deltaTicks / TICKS_PER_REV) * (60.0 / elapsed) / GEAR_RATIO;

        if (isMotor1) {
            lastPos1 = currentPos;
        } else {
            lastPos2 = currentPos;
        }

        velocityTimer.reset();
        return rpm;
    }

    // === PID CONTROLLER ===
    private static class ShooterPIDController {
        private double kP, kI, kD;
        private double lastError = 0;
        private double integral = 0;
        private long lastTime = 0;

        public void setPID(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }

        public double calculate(double current, double target) {
            long now = System.currentTimeMillis();
            double error = target - current;

            if (lastTime == 0) {
                lastTime = now;
                lastError = error;
                return 0;
            }

            double dt = (now - lastTime) / 1000.0;
            lastTime = now;

            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            return kP * error + kI * integral + kD * derivative;
        }

        public void reset() {
            lastError = 0;
            integral = 0;
            lastTime = 0;
        }
    }
}