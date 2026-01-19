package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;

    // Power values from EnhancedTele
    private static final double INTAKE_POWER = -1.0;
    private static final double INTAKE_TRANSFER_POWER = -0.15;

    private static final double SPIT_INTAKE_POWER = 0.9;
    private static final double SPIT_TRANSFER_POWER = -0.85;

    private static final double SHOOT_INTAKE_POWER = -0.55;
    private static final double SHOOT_TRANSFER_POWER = 0.5;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(INTAKE_TRANSFER_POWER);
    }

    public void runSpit() {
        intakeMotor.setPower(SPIT_INTAKE_POWER);
        transferMotor.setPower(SPIT_TRANSFER_POWER);
    }

    public void runShoot() {
        intakeMotor.setPower(SHOOT_INTAKE_POWER);
        transferMotor.setPower(SHOOT_TRANSFER_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    // Keep these methods as no-ops for compatibility with existing TeleOp code
    public void resetJam() {
        // No jam detection - method kept for compatibility
    }

    public boolean isJammed() {
        // Always return false since there's no jam detection
        return false;
    }

    public double getDistance() {
        // Return a safe default value
        return 999.0;
    }
}