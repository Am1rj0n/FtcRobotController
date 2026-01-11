package org.firstinspires.ftc.teamcode.subfilesV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LightingSubsystem {

    private final Servo shooterLight;
    private final Servo jamLight;

    private final ElapsedTime blinkTimer = new ElapsedTime();
    private boolean blinkState = false;

    private LightMode shooterMode = LightMode.RED;
    private LightMode jamMode = LightMode.GREEN;

    private static final double BLINK_RATE = 0.3;

    // REV Blinkin LED Driver positions
    private static final double RED = 0.61;     // Red
    private static final double GREEN = 0.77;   // Green
    private static final double BLUE = 0.87;    // Blue
    private static final double PINK = 0.57;    // Hot Pink
    private static final double OFF = 0.99;     // Black (off)

    public enum LightMode {
        RED, GREEN, BLUE, PINK,
        RED_BLINK, GREEN_BLINK, BLUE_BLINK, PINK_BLINK,
        OFF
    }

    public LightingSubsystem(HardwareMap hardwareMap) {
        shooterLight = hardwareMap.get(Servo.class, "shooter_light");
        jamLight = hardwareMap.get(Servo.class, "jam_light");

        blinkTimer.reset();
    }

    public void setShooterLight(LightMode mode) {
        shooterMode = mode;
    }

    public void setJamLight(LightMode mode) {
        jamMode = mode;
    }

    public void update() {
        // Handle blink timer
        if (blinkTimer.seconds() > BLINK_RATE) {
            blinkState = !blinkState;
            blinkTimer.reset();
        }

        // Apply shooter light
        shooterLight.setPosition(getLightPosition(shooterMode, blinkState));

        // Apply jam light
        jamLight.setPosition(getLightPosition(jamMode, blinkState));
    }

    private double getLightPosition(LightMode mode, boolean blinkOn) {
        switch (mode) {
            case RED:
                return RED;
            case GREEN:
                return GREEN;
            case BLUE:
                return BLUE;
            case PINK:
                return PINK;
            case RED_BLINK:
                return blinkOn ? RED : OFF;
            case GREEN_BLINK:
                return blinkOn ? GREEN : OFF;
            case BLUE_BLINK:
                return blinkOn ? BLUE : OFF;
            case PINK_BLINK:
                return blinkOn ? PINK : OFF;
            case OFF:
            default:
                return OFF;
        }
    }
}