package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeAutoPower {

    private DcMotor out1, out2;
    private DistanceHelper distanceHelper;

    // Default power when tag is not visible
    private final double DEFAULT_POWER = 0.34;

    public OuttakeAutoPower(DcMotor out1, DcMotor out2, DistanceHelper distanceHelper) {
        this.out1 = out1;
        this.out2 = out2;
        this.distanceHelper = distanceHelper;
    }

    /**
     * Returns the auto-calculated power based on distance
     */
    public double getAutoPower() {
        Double distance = distanceHelper.getDistanceMeters();
        if (distance == null) return DEFAULT_POWER;

        // Example mapping: distance (meters) -> power
        // Adjust these values for your field
        if (distance < 0.5) return 0.3;
        else if (distance < 1.0) return 0.335;
        else if (distance < 1.5) return 0.345;
        else if (distance < 1.7) return 0.350;
        else if (distance < 1.9) return 0.345;
        else if (distance < 2.1) return 0.37;
        else if (distance < 2.3) return 0.40;
        else if (distance < 2.5) return 0.43;
        else if (distance < 2.8) return 0.47;
        else if (distance < 3.2) return 0.60;
        else if (distance < 3.5) return 0.67;
        else if (distance < 3.7) return 0.72;
        else return DEFAULT_POWER; // Too far, use default
    }
}