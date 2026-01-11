package org.firstinspires.ftc.teamcode.tuners;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Auto PIDF Tuner", group = "Testing")
public class AUTOPIDTUNE extends LinearOpMode {

    // Shooter motors
    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;

    // PIDF values to tune
    private double kP = 0.001;
    private double kI = 0.0002;
    private double kD = 0.0001;
    private double kF = 0.0;

    // Target velocity for testing
    private double targetVelocity = 210.0;

    // Tuning mode
    private enum TuningParameter { P, I, D, F }
    private TuningParameter currentParam = TuningParameter.P;

    // Auto-tuning variables
    private boolean autoTuning = false;
    private ElapsedTime tuningTimer = new ElapsedTime();
    private double bestP = 0.005;
    private double bestError = Double.MAX_VALUE;
    private int tuningStep = 0;

    // Performance tracking
    private ElapsedTime performanceTimer = new ElapsedTime();
    private double totalError = 0;
    private int errorSamples = 0;

    // Button debouncing
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastStart = false;

    @Override
    public void runOpMode() {

        // === HARDWARE MAPPING ===
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        // Motor directions
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoder setup
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero power behavior
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Init telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== MANUAL TUNING CONTROLS ===");
        telemetry.addLine("Y: Switch parameter (P/I/D/F)");
        telemetry.addLine("Dpad Up/Down: Increase/Decrease by 0.001");
        telemetry.addLine("Dpad Left/Right: Increase/Decrease by 0.0001");
        telemetry.addLine("Left/Right Bumper: Target velocity ±10");
        telemetry.addLine();
        telemetry.addLine("=== AUTO TUNING ===");
        telemetry.addLine("START: Begin auto P-tuning");
        telemetry.addLine("(Tests P values to find best response)");
        telemetry.addLine();
        telemetry.addLine("A: Enable motors");
        telemetry.addLine("B: Disable motors");
        telemetry.update();

        waitForStart();

        boolean motorsEnabled = false;
        PIDFController pidf1 = null;
        PIDFController pidf2 = null;

        while (opModeIsActive()) {

            // === PARAMETER SELECTION ===
            boolean currentY = gamepad1.y;
            if (currentY && !lastY) {
                switch (currentParam) {
                    case P: currentParam = TuningParameter.I; break;
                    case I: currentParam = TuningParameter.D; break;
                    case D: currentParam = TuningParameter.F; break;
                    case F: currentParam = TuningParameter.P; break;
                }
            }
            lastY = currentY;

            // === MANUAL PARAMETER ADJUSTMENT ===
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            double largeStep = 0.001;
            double smallStep = 0.0001;

            if (dpadUp && !lastDpadUp) {
                adjustParameter(largeStep);
            }
            if (dpadDown && !lastDpadDown) {
                adjustParameter(-largeStep);
            }
            if (dpadRight && !lastDpadRight) {
                adjustParameter(smallStep);
            }
            if (dpadLeft && !lastDpadLeft) {
                adjustParameter(-smallStep);
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;

            // === TARGET VELOCITY ADJUSTMENT ===
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            if (lb && !lastLeftBumper) {
                targetVelocity = Math.max(0, targetVelocity - 10);
            }
            if (rb && !lastRightBumper) {
                targetVelocity = Math.min(400, targetVelocity + 10);
            }

            lastLeftBumper = lb;
            lastRightBumper = rb;

            // === MOTOR ENABLE/DISABLE ===
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;

            if (currentA && !lastA) {
                motorsEnabled = true;
                // Create new PIDF controllers with current values
                PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
                pidf1 = new PIDFController(coeffs);
                pidf2 = new PIDFController(coeffs);

                outtakeMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
                outtakeMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

                // Reset performance tracking
                performanceTimer.reset();
                totalError = 0;
                errorSamples = 0;
            }
            if (currentB && !lastB) {
                motorsEnabled = false;
                autoTuning = false;
            }

            lastA = currentA;
            lastB = currentB;

            // === AUTO TUNING START ===
            boolean currentStart = gamepad1.start;
            if (currentStart && !lastStart && !autoTuning) {
                startAutoTuning();
                motorsEnabled = true;
            }
            lastStart = currentStart;

            // === AUTO TUNING LOGIC ===
            if (autoTuning && motorsEnabled) {
                runAutoTuning(pidf1, pidf2);
            }

            // === MOTOR CONTROL ===
            if (motorsEnabled && pidf1 != null && pidf2 != null) {
                // Motor 1
                double currentVel1 = outtakeMotor1.getVelocity(AngleUnit.DEGREES);
                pidf1.updatePosition(currentVel1);
                pidf1.setTargetPosition(targetVelocity);
                double power1 = MathFunctions.clamp(pidf1.run(), -1, 1);
                outtakeMotor1.setPower(power1);

                // Motor 2
                double currentVel2 = outtakeMotor2.getVelocity(AngleUnit.DEGREES);
                pidf2.updatePosition(currentVel2);
                pidf2.setTargetPosition(targetVelocity);
                double power2 = MathFunctions.clamp(pidf2.run(), -1, 1);
                outtakeMotor2.setPower(power2);

                // Track performance
                double error1 = Math.abs(targetVelocity - currentVel1);
                double error2 = Math.abs(targetVelocity - currentVel2);
                double avgError = (error1 + error2) / 2.0;

                if (performanceTimer.seconds() > 1.0) { // Wait 1 second before tracking
                    totalError += avgError;
                    errorSamples++;
                }

                // === TELEMETRY ===
                telemetry.addLine("=== MOTOR STATUS ===");
                telemetry.addData("Motor 1 Vel", "%.1f / %.1f (err: %.1f)",
                        currentVel1, targetVelocity, error1);
                telemetry.addData("Motor 2 Vel", "%.1f / %.1f (err: %.1f)",
                        currentVel2, targetVelocity, error2);
                telemetry.addData("Avg Error", "%.2f deg/s", avgError);

                if (errorSamples > 0) {
                    telemetry.addData("Mean Error", "%.2f deg/s", totalError / errorSamples);
                }

            } else {
                outtakeMotor1.setPower(0);
                outtakeMotor2.setPower(0);
                telemetry.addLine("=== MOTORS DISABLED ===");
                telemetry.addData("Press A to enable", "");
            }

            telemetry.addLine();
            telemetry.addLine("=== PIDF VALUES ===");
            telemetry.addData("P", kP + (currentParam == TuningParameter.P ? " <--" : ""));
            telemetry.addData("I", kI + (currentParam == TuningParameter.I ? " <--" : ""));
            telemetry.addData("D", kD + (currentParam == TuningParameter.D ? " <--" : ""));
            telemetry.addData("F", kF + (currentParam == TuningParameter.F ? " <--" : ""));
            telemetry.addData("Target Velocity", "%.1f deg/s", targetVelocity);

            if (autoTuning) {
                telemetry.addLine();
                telemetry.addLine("=== AUTO TUNING ACTIVE ===");
                telemetry.addData("Step", "%d / 10", tuningStep);
                telemetry.addData("Testing P", "%.5f", kP);
                telemetry.addData("Best P so far", "%.5f", bestP);
                telemetry.addData("Best Error", "%.2f", bestError);
            }

            telemetry.update();
        }

        // Stop motors on exit
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    private void adjustParameter(double delta) {
        switch (currentParam) {
            case P:
                kP = Math.max(0, kP + delta);
                break;
            case I:
                kI = Math.max(0, kI + delta);
                break;
            case D:
                kD = Math.max(0, kD + delta);
                break;
            case F:
                kF = Math.max(0, kF + delta);
                break;
        }
    }

    private void startAutoTuning() {
        autoTuning = true;
        tuningStep = 0;
        bestP = kP;
        bestError = Double.MAX_VALUE;
        tuningTimer.reset();

        // Start with a low P value
        kP = 0.001;
        kI = 0;
        kD = 0;
        kF = 0;
    }

    private void runAutoTuning(PIDFController pidf1, PIDFController pidf2) {
        // Each tuning step lasts 3 seconds
        if (tuningTimer.seconds() < 3.0) {
            return; // Still testing current value
        }

        // Calculate average error for this test
        double avgError = errorSamples > 0 ? totalError / errorSamples : Double.MAX_VALUE;

        // Check if this is the best so far
        if (avgError < bestError) {
            bestError = avgError;
            bestP = kP;
        }

        // Move to next step
        tuningStep++;

        if (tuningStep >= 10) {
            // Tuning complete
            kP = bestP;
            autoTuning = false;
            return;
        }

        // Try next P value (increase by 0.002 each step)
        kP += 0.002;

        // Reset for next test
        PIDFCoefficients coeffs = new PIDFCoefficients(kP, kI, kD, kF);
        pidf1 = new PIDFController(coeffs);
        pidf2 = new PIDFController(coeffs);

        outtakeMotor1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        outtakeMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        tuningTimer.reset();
        performanceTimer.reset();
        totalError = 0;
        errorSamples = 0;
    }
}