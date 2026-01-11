package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Enhanced Shooter Test (Power)", group = "Testing")
public class OUTTEST extends LinearOpMode {

    // Outtake motors
    private DcMotorEx outtakeMotor1;
    private DcMotorEx outtakeMotor2;

    // Intake motors (for shooting mode)
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    // Shot powers (adjustable)
    private double closePower1 = 0.46;   // Bottom motor close
    private double closePower2 = 0.57;   // Top motor close
    private double farPower1 = 0.7;      // Bottom motor far
    private double farPower2 = 0.8;      // Top motor far

    // Operating mode
    private enum OperatingMode {
        IDLE,
        CLOSE_SHOT,
        FAR_SHOT,
        SHOOTING,
        INTAKE
    }
    private OperatingMode currentMode = OperatingMode.IDLE;

    // Button state - GAMEPAD 1
    private boolean lastA_gp1 = false;
    private boolean lastB_gp1 = false;
    private boolean lastX_gp1 = false;
    private boolean lastY_gp1 = false;
    private boolean lastDpadUp_gp1 = false;
    private boolean lastDpadDown_gp1 = false;
    private boolean lastDpadLeft_gp1 = false;
    private boolean lastDpadRight_gp1 = false;
    private boolean lastRightBumper_gp1 = false;

    @Override
    public void runOpMode() {

        // Hardware mapping
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");

        // Motor direction
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // FLOAT mode for shooters
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // BRAKE mode for intake
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("=== GAMEPAD 1 - SHOOTING ===");
        telemetry.addLine("  A: Close Shot Mode");
        telemetry.addLine("  B: Far Shot Mode");
        telemetry.addLine("  X: Intake Mode");
        telemetry.addLine("  Y: Stop All");
        telemetry.addLine("  Right Trigger: SHOOT");
        telemetry.addLine("  Dpad Up/Down: Adjust Motor 1 Power (±0.01)");
        telemetry.addLine("  Dpad Left/Right: Adjust Motor 2 Power (±0.01)");
        telemetry.addLine("  Right Bumper: Toggle Mode (Close/Far)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === GAMEPAD 1: SHOOTING CONTROLS ===
            handleShootingControls();

            // === UPDATE MOTORS ===
            updateShooter();
            updateIntake();

            // === TELEMETRY ===
            displayTelemetry();
        }

        // Stop all motors
        stopAll();
    }

    /**
     * GAMEPAD 1: Shooting and mode controls
     */
    private void handleShootingControls() {
        // Mode selection
        if (gamepad1.a && !lastA_gp1) {
            currentMode = OperatingMode.CLOSE_SHOT;
        }
        if (gamepad1.b && !lastB_gp1) {
            currentMode = OperatingMode.FAR_SHOT;
        }
        if (gamepad1.x && !lastX_gp1) {
            currentMode = OperatingMode.INTAKE;
        }
        if (gamepad1.y && !lastY_gp1) {
            currentMode = OperatingMode.IDLE;
        }

        // Toggle between close and far shot
        if (gamepad1.right_bumper && !lastRightBumper_gp1) {
            if (currentMode == OperatingMode.CLOSE_SHOT) {
                currentMode = OperatingMode.FAR_SHOT;
            } else if (currentMode == OperatingMode.FAR_SHOT) {
                currentMode = OperatingMode.CLOSE_SHOT;
            }
        }

        // Shoot trigger
        if (gamepad1.right_trigger > 0.5) {
            if (currentMode == OperatingMode.CLOSE_SHOT || currentMode == OperatingMode.FAR_SHOT) {
                currentMode = OperatingMode.SHOOTING;
            }
        } else {
            if (currentMode == OperatingMode.SHOOTING) {
                // Return to previous shot mode
                currentMode = OperatingMode.CLOSE_SHOT;
            }
        }

        // Power adjustments for current mode (±0.01)
        if (currentMode == OperatingMode.CLOSE_SHOT || currentMode == OperatingMode.FAR_SHOT) {
            // Motor 1 power adjustment
            if (gamepad1.dpad_up && !lastDpadUp_gp1) {
                if (currentMode == OperatingMode.CLOSE_SHOT) {
                    closePower1 = Math.min(closePower1 + 0.01, 1.0);
                } else {
                    farPower1 = Math.min(farPower1 + 0.01, 1.0);
                }
            }
            if (gamepad1.dpad_down && !lastDpadDown_gp1) {
                if (currentMode == OperatingMode.CLOSE_SHOT) {
                    closePower1 = Math.max(closePower1 - 0.01, 0.0);
                } else {
                    farPower1 = Math.max(farPower1 - 0.01, 0.0);
                }
            }

            // Motor 2 power adjustment
            if (gamepad1.dpad_right && !lastDpadRight_gp1) {
                if (currentMode == OperatingMode.CLOSE_SHOT) {
                    closePower2 = Math.min(closePower2 + 0.01, 1.0);
                } else {
                    farPower2 = Math.min(farPower2 + 0.01, 1.0);
                }
            }
            if (gamepad1.dpad_left && !lastDpadLeft_gp1) {
                if (currentMode == OperatingMode.CLOSE_SHOT) {
                    closePower2 = Math.max(closePower2 - 0.01, 0.0);
                } else {
                    farPower2 = Math.max(farPower2 - 0.01, 0.0);
                }
            }
        }

        // Update button states
        lastA_gp1 = gamepad1.a;
        lastB_gp1 = gamepad1.b;
        lastX_gp1 = gamepad1.x;
        lastY_gp1 = gamepad1.y;
        lastDpadUp_gp1 = gamepad1.dpad_up;
        lastDpadDown_gp1 = gamepad1.dpad_down;
        lastDpadLeft_gp1 = gamepad1.dpad_left;
        lastDpadRight_gp1 = gamepad1.dpad_right;
        lastRightBumper_gp1 = gamepad1.right_bumper;
    }

    /**
     * Update shooter motors based on current mode
     */
    private void updateShooter() {
        switch (currentMode) {
            case IDLE:
                outtakeMotor1.setPower(0);
                outtakeMotor2.setPower(0);
                break;

            case CLOSE_SHOT:
                outtakeMotor1.setPower(closePower1);
                outtakeMotor2.setPower(closePower2);
                break;

            case FAR_SHOT:
                outtakeMotor1.setPower(farPower1);
                outtakeMotor2.setPower(farPower2);
                break;

            case SHOOTING:
                // Keep shooter running at current target
                if (closePower1 > farPower1) { // Was in close shot mode
                    outtakeMotor1.setPower(closePower1);
                    outtakeMotor2.setPower(closePower2);
                } else { // Was in far shot mode
                    outtakeMotor1.setPower(farPower1);
                    outtakeMotor2.setPower(farPower2);
                }
                break;

            case INTAKE:
                outtakeMotor1.setPower(0);
                outtakeMotor2.setPower(0);
                break;
        }
    }

    /**
     * Update intake/transfer motors
     */
    private void updateIntake() {
        switch (currentMode) {
            case INTAKE:
                intakeMotor.setPower(-1.0);
                transferMotor.setPower(-0.15);
                break;

            case SHOOTING:
                intakeMotor.setPower(-0.55);
                transferMotor.setPower(0.55);
                break;

            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }

    /**
     * Display telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("=== CURRENT MODE ===");
        telemetry.addData("Mode", currentMode.name());
        telemetry.addLine();

        telemetry.addLine("=== MOTOR 1 (Bottom) ===");
        telemetry.addData("Current Power", "%.2f", outtakeMotor1.getPower());
        telemetry.addData("Close Power", "%.2f (%.0f%%)", closePower1, closePower1 * 100);
        telemetry.addData("Far Power", "%.2f (%.0f%%)", farPower1, farPower1 * 100);
        telemetry.addLine();

        telemetry.addLine("=== MOTOR 2 (Top) ===");
        telemetry.addData("Current Power", "%.2f", outtakeMotor2.getPower());
        telemetry.addData("Close Power", "%.2f (%.0f%%)", closePower2, closePower2 * 100);
        telemetry.addData("Far Power", "%.2f (%.0f%%)", farPower2, farPower2 * 100);
        telemetry.addLine();

        telemetry.addLine("=== INTAKE SYSTEM ===");
        telemetry.addData("Intake Motor", "%.2f", intakeMotor.getPower());
        telemetry.addData("Transfer Motor", "%.2f", transferMotor.getPower());

        telemetry.update();
    }

    /**
     * Stop all motors
     */
    private void stopAll() {
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }
}