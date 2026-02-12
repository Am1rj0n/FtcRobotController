package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;

    // Motor speeds for different modes
    private static final double INTAKE_SPEED = 0.9;
    private static final double TRANSFER_INTAKE_SPEED = -0.1;
    private static final double SPIT_SPEED = -0.8;
    private static final double SHOOT_SPEED = 1.0;

    public enum Mode {
        OFF,
        INTAKE,
        SPIT,
        SHOOT
    }

    private Mode currentMode = Mode.OFF;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        transferMotor = hardwareMap.dcMotor.get("transfer");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(Mode mode) {
        currentMode = mode;
        applyMode();
    }

    private void applyMode() {
        switch (currentMode) {
            case INTAKE:
                intakeMotor.setPower(INTAKE_SPEED);
                transferMotor.setPower(TRANSFER_INTAKE_SPEED);
                break;

            case SPIT:
                intakeMotor.setPower(SPIT_SPEED);
                transferMotor.setPower(SPIT_SPEED);
                break;

            case SHOOT:
                intakeMotor.setPower(SHOOT_SPEED);
                transferMotor.setPower(SHOOT_SPEED);
                break;

            case OFF:
            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }

    public void stop() {
        setMode(Mode.OFF);
    }

    public Mode getCurrentMode() {
        return currentMode;
    }
}