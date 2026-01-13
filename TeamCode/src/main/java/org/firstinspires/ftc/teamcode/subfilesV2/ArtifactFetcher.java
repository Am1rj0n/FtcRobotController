package org.firstinspires.ftc.teamcode.subfilesV2;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BlobCamera;
import org.firstinspires.ftc.teamcode.vision.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.vision.ColorRange;
import org.firstinspires.ftc.teamcode.vision.ImageRegion;

import java.util.List;

public class ArtifactFetcher {

    private final Follower follower;
    private final ColorBlobLocatorProcessor purpleLocator;
    private final ColorBlobLocatorProcessor greenLocator;

    private final PIDFController rotController;
    private final PIDFController yController;
    private final PIDFController xController;

    private boolean active = false;
    private boolean hasTarget = false;

    // PID constants - Adjusted for smoother approach
    private static final double Pr = 0.003;
    private static final double Dr = 0.0003;
    private static final double Py = 0.002;
    private static final double Dy = 0.0002;
    private static final double Px = 0.002;
    private static final double Dx = 0.0002;

    // Vision constants
    private static final double CAM_CENTER = 160;   // Center of 320px width
    private static final double BLOB_RAD_GOAL = 200; // Radius when ball is at intake

    public ArtifactFetcher(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();

        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .build();

        BlobCamera portal = new BlobCamera.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(greenLocator, purpleLocator)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(BlobCamera.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        rotController = new PIDFController(new PIDFCoefficients(Pr, 0, Dr, 0));
        xController = new PIDFController(new PIDFCoefficients(Px, 0, Dx, 0));
        yController = new PIDFController(new PIDFCoefficients(Py, 0, Dy, 0));
    }

    public void toggle() {
        active = !active;
        if (!active) {
            stopMovement();
        }
    }

    public void update() {
        if (!active) {
            hasTarget = false;
            return;
        }

        List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
        blobs.addAll(greenLocator.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 100, 20000, blobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.5, 1, blobs);

        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        if (!blobs.isEmpty()) {
            hasTarget = true;
            ColorBlobLocatorProcessor.Blob target = blobs.get(0);

            // --- ERROR CALCULATIONS ---

            // 1. Rotation: (Center - CurrentX).
            // If ball is at 200, Er = 160 - 200 = -40 (Turn right)
            // 1. Rotation: Back to original (Center - Current)
            // 1. Rotation: Back to original (Center - Current)
            double Er = CAM_CENTER - target.getCircle().getX();

// 2. Forward/Back: Keep the flip (Goal - Current)
// This ensures: Radius 50 (far) -> 200 - 50 = +150 error -> Move Forward
            double Ey = BLOB_RAD_GOAL - target.getCircle().getRadius();
            // Update PID Controllers
            rotController.updateError(Er);
            yController.updateError(Ey);
            xController.updateError(0); // Keeping strafe at 0 for stability

            // --- DRIVE COMMAND ---
            // Pedro Pathing setTeleOpDrive(Forward/Back, Strafe, Turn, isRobotCentric)
            follower.setTeleOpDrive(
                    yController.run(),   // Forward Power
                    0,                   // Strafe Power (set to 0 to prevent sliding)
                    rotController.run(), // Turn Power
                    true                 // Robot Centric is REQUIRED for this
            );
        } else {
            hasTarget = false;
            stopMovement();
        }
    }

    private void stopMovement() {
        rotController.updateError(0);
        yController.updateError(0);
        xController.updateError(0);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    public boolean isActive() { return active; }
    public boolean hasTarget() { return hasTarget; }
}