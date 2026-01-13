package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;

/**
 * Dual-motor vertical shooter using PedroPathing PIDF controllers
 * - Motor 1 (bottom): Different velocity
 * - Motor 2 (top): Different velocity for spin control
 */
public class DualMotorShooterHelper {

    private DcMotorEx outtakeMotor1; // Bottom motor
    private DcMotorEx outtakeMotor2; // Top motor

    // PedroPathing PIDF Controllers
    private PIDFController pidfMotor1;
    private PIDFController pidfMotor2;

    // Velocity targets in degrees per second
    private static final double TICKS_PER_REV = 28.0;

    // Shot powers (adjustable)
    public double closeMoitor1Power = 0.55; // Bottom motor close
    public double closeMotor2Power = 0.70; // Top motor close
    public double farMotor1Power = 0.67;   // Bottom motor far
    public double farMotor2Power = 0.67;   // Top motor far

    // Max velocity in degrees/sec
    // 5800 RPM = 5800 * 6 = 34,800 deg/min = 580 deg/sec
    private static final double MAX_VELOCITY_DEG_SEC = (5800.0 * 360.0) / 60.0; // ~34,800 deg/sec

    // Velocity tolerance for "at target" detection (degrees/sec)
    private static final double VELOCITY_TOLERANCE = 500.0; // Within 500 deg/sec

    // Current target velocities
    private double targetVelocity1 = 0;
    private double targetVelocity2 = 0;

    public DualMotorShooterHelper(HardwareMap hardwareMap, PIDFCoefficients shooterCoefficients) {
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        // Configure motors for velocity control
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOAT mode for shooters
        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Initialize PedroPathing PIDF controllers
        pidfMotor1 = new PIDFController(shooterCoefficients);
        pidfMotor2 = new PIDFController(shooterCoefficients);
    }

    /**
     * Update shooter velocities with PIDF control
     * Call this every loop iteration
     */
    public void update() {
        if (targetVelocity1 > 0 || targetVelocity2 > 0) {
            // Get current velocities
            double currentVelocity1 = outtakeMotor1.getVelocity(AngleUnit.DEGREES);
            double currentVelocity2 = outtakeMotor2.getVelocity(AngleUnit.DEGREES);

            // Update PIDF positions
            pidfMotor1.updatePosition(currentVelocity1);
            pidfMotor2.updatePosition(currentVelocity2);

            // Set targets
            pidfMotor1.setTargetPosition(targetVelocity1);
            pidfMotor2.setTargetPosition(targetVelocity2);

            // Calculate and apply power using Pedro's run() method
            double power1 = pidfMotor1.run();
            double power2 = pidfMotor2.run();

            outtakeMotor1.setPower(Math.max(-1.0, Math.min(1.0, power1)));
            outtakeMotor2.setPower(Math.max(-1.0, Math.min(1.0, power2)));
        }
    }

    /**
     * Set target velocities based on power percentages
     */
    private void setTargetVelocities(double motor1Power, double motor2Power) {
        targetVelocity1 = motor1Power * MAX_VELOCITY_DEG_SEC;
        targetVelocity2 = motor2Power * MAX_VELOCITY_DEG_SEC;
    }

    /**
     * Run close shot
     */
    public void runCloseShot() {
        setTargetVelocities(closeMoitor1Power, closeMotor2Power);
    }

    /**
     * Run far shot
     */
    public void runFarShot() {
        setTargetVelocities(farMotor1Power, farMotor2Power);
    }

    /**
     * Adjust both close and far shot powers
     */
    public void adjustPowers(double delta) {
        closeMoitor1Power = Math.max(0, Math.min(1.0, closeMoitor1Power + delta));
        closeMotor2Power = Math.max(0, Math.min(1.0, closeMotor2Power + delta));
        farMotor1Power = Math.max(0, Math.min(1.0, farMotor1Power + delta));
        farMotor2Power = Math.max(0, Math.min(1.0, farMotor2Power + delta));
    }

    /**
     * Check if shooter has reached target velocity
     */
    public boolean isAtTargetVelocity() {
        if (targetVelocity1 == 0 && targetVelocity2 == 0) return false;

        double currentVelocity1 = outtakeMotor1.getVelocity(AngleUnit.DEGREES);
        double currentVelocity2 = outtakeMotor2.getVelocity(AngleUnit.DEGREES);

        boolean motor1Ready = Math.abs(currentVelocity1 - targetVelocity1) < VELOCITY_TOLERANCE;
        boolean motor2Ready = Math.abs(currentVelocity2 - targetVelocity2) < VELOCITY_TOLERANCE;

        return motor1Ready && motor2Ready;
    }

    /**
     * Stop shooter
     */
    public void stop() {
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
        targetVelocity1 = 0;
        targetVelocity2 = 0;
    }

    /**
     * Get current velocities for telemetry (in degrees/second)
     */
    public double getMotor1Velocity() {
        return outtakeMotor1.getVelocity(AngleUnit.DEGREES);
    }

    public double getMotor2Velocity() {
        return outtakeMotor2.getVelocity(AngleUnit.DEGREES);
    }

    /**
     * Get target velocities for telemetry
     */
    public double getTargetMotor1Velocity(ShooterMode mode) {
        double power = (mode == ShooterMode.CLOSE) ? closeMoitor1Power : farMotor1Power;
        return power * MAX_VELOCITY_DEG_SEC;
    }

    public double getTargetMotor2Velocity(ShooterMode mode) {
        double power = (mode == ShooterMode.CLOSE) ? closeMotor2Power : farMotor2Power;
        return power * MAX_VELOCITY_DEG_SEC;
    }

    public enum ShooterMode { CLOSE, FAR }
}