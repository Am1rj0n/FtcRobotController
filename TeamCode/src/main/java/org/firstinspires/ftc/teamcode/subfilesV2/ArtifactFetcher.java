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

    private boolean active = false;
    private boolean hasTarget = false;

    // FIXED: Inverted PID constants for correct direction
    private static final double Pr = 0.004;   // Increased for faster response
    private static final double Dr = 0.0005;
    private static final double Py = 0.0025;  // Increased forward drive
    private static final double Dy = 0.0003;

    // Vision constants
    private static final double CAM_CENTER = 160;
    private static final double BLOB_RAD_GOAL = 200;

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

            // FIXED ERROR CALCULATIONS:
            // Rotation: Ball right of center (X=200) means turn RIGHT (negative turn)
            // Error = Current - Center (so positive X -> negative turn)
            double Er = target.getCircle().getX() - CAM_CENTER;

            // Forward: Small radius (far) means move FORWARD (positive)
            // Error = Current - Goal (so small radius -> negative error -> positive output)
            double Ey = target.getCircle().getRadius() - BLOB_RAD_GOAL;

            // Update controllers
            rotController.updateError(Er);
            yController.updateError(Ey);

            // FIXED: Corrected sign conventions
            // Negative turn = rotate right, Positive forward = drive forward
            follower.setTeleOpDrive(
                    yController.run(),  // FORWARD (inverted to match expected direction)
                    0,                   // No strafe
                    -rotController.run(), // TURN (inverted to match expected direction)
                    true                 // Robot-centric
            );
        } else {
            hasTarget = false;
            stopMovement();
        }
    }

    private void stopMovement() {
        rotController.updateError(0);
        yController.updateError(0);
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    public boolean isActive() { return active; }
    public boolean hasTarget() { return hasTarget; }
}