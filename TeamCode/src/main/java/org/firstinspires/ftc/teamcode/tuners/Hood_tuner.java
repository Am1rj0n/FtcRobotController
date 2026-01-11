package org.firstinspires.ftc.teamcode.tuners;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Hood PID Tuner", group="Tuning")
public class Hood_tuner extends OpMode {

    private DcMotorEx hoodMotor;
    private HoodPIDController controller;

    // Tunable PID values - adjust with gamepad
    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double TARGET_POSITION = 0; // Target encoder position

    // Motor specs - 6000 RPM motor, using up to 5500 RPM
    private static final double TICKS_PER_REV = 28.0; // Adjust for your motor
    private static final double GEAR_RATIO = 1.0; // Adjust if geared
    private static final double MAX_RPM = 6000.0;
    private static final double TARGET_MAX_RPM = 5500.0; // Using 5500 out of 6000 available

    // Position limits (adjust based on your hood's physical range)
    private static final double MIN_POSITION = 0;
    private static final double MAX_POSITION = 1000; // Adjust to your hood's max encoder value

    // Fine/coarse adjustment modes
    private boolean fineAdjustMode = false;
    private double positionStep = 50; // Coarse adjustment
    private double finePositionStep = 5; // Fine adjustment

    @Override
    public void init() {
        controller = new HoodPIDController();

        hoodMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");

        // Motor setup - NOT reversed
        hoodMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoder and set to position mode
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake mode for precise positioning
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TARGET_POSITION = hoodMotor.getCurrentPosition();

        telemetry.addLine("=== HOOD PID TUNING ===");
        telemetry.addLine();
        telemetry.addLine("GAMEPAD CONTROLS:");
        telemetry.addLine("  D-Pad Up/Down: P ±0.001");
        telemetry.addLine("  D-Pad Right/Left: D ±0.0001");
        telemetry.addLine("  Y/A: I ±0.00001");
        telemetry.addLine("  RB/LB: Target Position ±step");
        telemetry.addLine("  RT: Toggle Fine/Coarse (step size)");
        telemetry.addLine("  X: Reset PID + Go to 0");
        telemetry.addLine("  B: Emergency Stop");
        telemetry.addLine();
        telemetry.addLine("Position control with encoder feedback");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === GAMEPAD CONTROLS FOR TUNING ===
        if (gamepad1.dpad_up) P += 0.001;
        if (gamepad1.dpad_down) P = Math.max(0, P - 0.001);
        if (gamepad1.dpad_right) D += 0.0001;
        if (gamepad1.dpad_left) D = Math.max(0, D - 0.0001);
        if (gamepad1.y) I += 0.00001;
        if (gamepad1.a) I = Math.max(0, I - 0.00001);

        // Toggle fine/coarse adjustment
        if (gamepad1.right_trigger > 0.5) {
            fineAdjustMode = !fineAdjustMode;
            while (gamepad1.right_trigger > 0.5) {} // Wait for release
        }

        double currentStep = fineAdjustMode ? finePositionStep : positionStep;

        if (gamepad1.right_bumper) {
            TARGET_POSITION = Math.min(MAX_POSITION, TARGET_POSITION + currentStep);
        }
        if (gamepad1.left_bumper) {
            TARGET_POSITION = Math.max(MIN_POSITION, TARGET_POSITION - currentStep);
        }




        // === GET CURRENT POSITION ===
        double currentPosition = hoodMotor.getCurrentPosition();

        // === UPDATE PID ===
        controller.setPID(P, I, D);

        // === CALCULATE PID OUTPUT ===
        double pidOutput = controller.calculate(currentPosition, TARGET_POSITION);

        // === CONVERT TO POWER (-1 to 1) ===
        // Scale PID output to reasonable power range
        double power = pidOutput / 1000.0; // Adjust this divisor based on your system
        power = Math.max(-1.0, Math.min(1.0, power)); // Clamp -1 to 1

        // === APPLY POWER ===
        hoodMotor.setPower(power);

        // === CALCULATE ERROR ===
        double error = Math.abs(TARGET_POSITION - currentPosition);
        double percentError = (TARGET_POSITION != 0) ? (error / Math.abs(TARGET_POSITION) * 100.0) : 0;

        // === TELEMETRY ===
        telemetry.addLine("=== GAMEPAD CONTROLS ===");
        telemetry.addLine("D-Pad Up/Down: P | D-Pad L/R: D");
        telemetry.addLine("Y/A: I | RB/LB: Target | RT: Mode | X: Reset");
        telemetry.addLine();

        telemetry.addData("=== PID VALUES ===", "");
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("I", "%.6f", I);
        telemetry.addData("D", "%.6f", D);
        telemetry.addLine();

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("Target", "%.0f ticks", TARGET_POSITION);
        telemetry.addData("Current", "%.0f ticks", currentPosition);
        telemetry.addData("Error", "%.0f ticks (%.1f%%)", error, percentError);
        telemetry.addLine();

        telemetry.addData("=== MODE ===", "");
        telemetry.addData("Adjustment", fineAdjustMode ? "FINE (±5)" : "COARSE (±50)");
        telemetry.addData("Step Size", "%.0f ticks", currentStep);
        telemetry.addLine();

        telemetry.addData("=== OUTPUT ===", "");
        telemetry.addData("PID Output", "%.1f", pidOutput);
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addLine();

        // === TUNING STATUS ===
        String status;
        if (error < 5) {
            status = "✓ EXCELLENT - On Target!";
        } else if (error < 15) {
            status = "○ GOOD - Close to target";
        } else if (error < 50) {
            status = "△ FAIR - Needs tuning";
        } else {
            status = "✗ POOR - Adjust PID values";
        }
        telemetry.addData("Status", status);

        telemetry.update();
    }

    @Override
    public void stop() {
        hoodMotor.setPower(0);
    }

    // Inner PID Controller class for position control
    private class HoodPIDController {
        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;

        private double lastError = 0.0;
        private double integral = 0.0;
        private long lastTime = 0;

        public void setPID(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }

        public double calculate(double current, double target) {
            long currentTime = System.currentTimeMillis();
            double error = target - current;

            if (lastTime == 0) {
                lastTime = currentTime;
                lastError = error;
                return 0.0;
            }

            double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds

            if (dt > 0) {
                integral += error * dt;

                // Anti-windup: limit integral accumulation
                integral = Math.max(-500, Math.min(500, integral));

                double derivative = (error - lastError) / dt;

                lastError = error;
                lastTime = currentTime;

                return kP * error + kI * integral + kD * derivative;
            }

            return 0.0;
        }

        public void reset() {
            lastError = 0.0;
            integral = 0.0;
            lastTime = 0;
        }
    }
}