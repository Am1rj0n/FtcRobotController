package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Test: Shooter", group = "Tests")
public class TestShooter extends OpMode {

    private Shooter shooter;

    private boolean lastSquare = false;
    private boolean lastTriangle = false;
    private boolean lastL1 = false;
    private boolean lastR1 = false;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);

        telemetry.addLine("Shooter Test Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Square: Toggle ON/OFF");
        telemetry.addLine("  Triangle: Toggle Close/Far");
        telemetry.addLine("  L1: Decrease RPM");
        telemetry.addLine("  R1: Increase RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle shooter
        if (gamepad1.square && !lastSquare) {
            shooter.toggle();
        }
        lastSquare = gamepad1.square;

        // Toggle mode
        if (gamepad1.triangle && !lastTriangle) {
            shooter.toggleMode();
        }
        lastTriangle = gamepad1.triangle;

        // RPM adjustment
        if (gamepad1.left_bumper && !lastL1) {
            shooter.decreaseRPM();
        }
        if (gamepad1.right_bumper && !lastR1) {
            shooter.increaseRPM();
        }
        lastL1 = gamepad1.left_bumper;
        lastR1 = gamepad1.right_bumper;

        // CRITICAL: Update shooter
        shooter.periodic();

        telemetry.addLine("╔═══ SHOOTER TEST ═══╗");
        telemetry.addData("│ %s", shooter.getTelemetryString());
        telemetry.addLine("╚═══════════════════╝");
        telemetry.update();
    }
}