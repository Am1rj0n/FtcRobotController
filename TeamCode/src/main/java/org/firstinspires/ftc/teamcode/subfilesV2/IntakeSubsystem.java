package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final DistanceSensor intakeSensor;

    private final ElapsedTime jamTimer = new ElapsedTime();
    private boolean objectDetected = false;
    private boolean jammed = false;

    private static final double JAM_DISTANCE_CM = 17.0;
    private static final double JAM_TIME = 0.7;
    // Add clearance threshold - object must move away before unjamming
    private static final double CLEAR_DISTANCE_CM = 20.0;

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

        intakeSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");

        jamTimer.reset();
    }

    public void runIntake() {
        // Update jam detection ONCE per call
        updateJamDetection();

        if (!jammed) {
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(INTAKE_TRANSFER_POWER);
        } else {
            stop();
        }
    }

    public void runSpit() {
        // Clear jam status when spitting
        resetJam();
        intakeMotor.setPower(SPIT_INTAKE_POWER);
        transferMotor.setPower(SPIT_TRANSFER_POWER);
    }

    public void runShoot() {
        // Clear jam status when shooting
        resetJam();
        intakeMotor.setPower(SHOOT_INTAKE_POWER);
        transferMotor.setPower(SHOOT_TRANSFER_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }

    private void updateJamDetection() {
        double distance = intakeSensor.getDistance(DistanceUnit.CM);

        // FIXED: Two-threshold system for jam detection
        if (distance < JAM_DISTANCE_CM) {
            if (!objectDetected) {
                objectDetected = true;
                jamTimer.reset();
            } else if (jamTimer.seconds() > JAM_TIME && !jammed) {
                jammed = true;
            }
        } else if (distance > CLEAR_DISTANCE_CM) {
            // Only clear jam when object is fully away (20cm+)
            objectDetected = false;
            jammed = false;
            jamTimer.reset();
        }
        // Between 17-20cm: maintain current state (hysteresis)
    }

    public void resetJam() {
        this.jammed = false;
        this.objectDetected = false;
        jamTimer.reset();
    }

    public boolean isJammed() {
        return jammed;
    }

    public double getDistance() {
        return intakeSensor.getDistance(DistanceUnit.CM);
    }
}