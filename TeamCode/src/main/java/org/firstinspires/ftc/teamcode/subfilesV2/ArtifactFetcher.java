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

    // PID constants
    private static final double Pr = 0.003;
    private static final double Dr = 0.0003;
    private static final double Py = 0.002;
    private static final double Dy = 0.0002;
    private static final double Px = 0.002;
    private static final double Dx = 0.0002;

    // Vision constants
    private static final double CAM_CENTER = 160;
    private static final double BLOB_RAD_GOAL = 200;

    public ArtifactFetcher(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        // Initialize purple locator
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Initialize green locator
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Initialize camera
        BlobCamera portal = new BlobCamera.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(greenLocator, purpleLocator)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(BlobCamera.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        // Initialize PID controllers
        rotController = new PIDFController(new PIDFCoefficients(Pr, 0, Dr, 0));
        xController = new PIDFController(new PIDFCoefficients(Px, 0, Dx, 0));
        yController = new PIDFController(new PIDFCoefficients(Py, 0, Dy, 0));
    }

    public void toggle() {
        active = !active;
        if (!active) {
            // Reset controllers when disabled
            rotController.updateError(0);
            yController.updateError(0);
            xController.updateError(0);
        }
    }

    public void update() {
        if (!active) {
            hasTarget = false;
            return;
        }

        // Get all blobs (purple + green)
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleLocator.getBlobs();
        blobs.addAll(greenLocator.getBlobs());

        // Filter by size and circularity
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                100, 20000, blobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.5, 1, blobs);

        // Sort by size (largest first)
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                SortOrder.DESCENDING, blobs);

        if (!blobs.isEmpty()) {
            hasTarget = true;

            // Get errors
            double Er = CAM_CENTER - blobs.get(0).getCircle().getX();
            double Ey = BLOB_RAD_GOAL - blobs.get(0).getCircle().getRadius();
            double Ex = CAM_CENTER - blobs.get(0).getCircle().getX();

            // Update controllers
            rotController.updateError(Er);
            yController.updateError(Ey);
            xController.updateError(Ex);

            // Drive toward artifact (robot-centric)
            follower.setTeleOpDrive(
                    yController.run(),
                    xController.run(),
                    rotController.run(),
                    true // Robot-centric
            );
        } else {
            hasTarget = false;
            rotController.updateError(0);
            yController.updateError(0);
            xController.updateError(0);
        }
    }

    public boolean isActive() {
        return active;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
}