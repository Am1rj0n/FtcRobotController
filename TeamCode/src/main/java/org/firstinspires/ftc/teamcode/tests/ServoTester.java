package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester - Stick + Reverse Toggle", group = "Testing")
public class ServoTester extends LinearOpMode {

    private Servo testServo;

    private double pos = 0.5;   // raw position (0–1)
    private final double STEP = 0.1;   // MUCH smaller step

    private boolean reversed = false;
    private boolean lastAToggle = false;

    @Override
    public void runOpMode() {

        testServo = hardwareMap.get(Servo.class, "transfer_servo");

        waitForStart();

        while (opModeIsActive()) {

            // --- REVERSE TOGGLE ---
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAToggle) {
                reversed = !reversed;
            }
            lastAToggle = aPressed;

            // --- SERVO POSITION CONTROL (LEFT STICK Y) ---
            double stick = -gamepad1.left_stick_y;

            // deadband
            if (Math.abs(stick) > 0.1) {
                pos += stick * STEP;
            }

            // Clamp between 0–1
            pos = Math.max(0, Math.min(1, pos));

            // If reversed, invert the output
            double finalPos = reversed ? (1 - pos) : pos;

            // Send to servo
            testServo.setPosition(finalPos);

            // --- TELEMETRY ---
            telemetry.addLine("=== SERVO TESTER ===");
            telemetry.addData("Raw Pos (0–1)", "%.3f", pos);
            telemetry.addData("Final Pos", "%.3f", finalPos);
            telemetry.addData("Reversed", reversed ? "YES" : "NO");
            telemetry.addLine("Controls:");
            telemetry.addLine("Left Stick = Move Servo");
            telemetry.addLine("A = Toggle Reverse");
            telemetry.update();

            sleep(15); // smoother stepping
        }
    }
}
