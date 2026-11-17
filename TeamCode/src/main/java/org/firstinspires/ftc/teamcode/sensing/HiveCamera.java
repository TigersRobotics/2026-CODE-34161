package org.firstinspires.ftc.teamcode.sensing;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.SensingConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Owns the webcam and the AprilTag detector.
 * <p>
 * The detector runs on the SDK's own camera thread. The OpMode loop only ever asks "is there a new
 * frame?" ({@link #poll()}), which costs almost nothing, so the camera never slows the loop down.
 * Only the four BIOBUZZ hive clusters are loaded, the preview is off unless asked for, and the
 * exposure is fixed short so the tags stay sharp while the robot moves.
 */
final class HiveCamera {
    // SDK names of the four clusters, in the order SensingConstants numbers them
    private static final String[] CLUSTER_NAMES = { "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING" };
    private static final long EXPOSURE_STEP_GAP_NS = 60_000_000L; // the camera needs a moment between settings

    private final VisionPortal portal;
    private final AprilTagProcessor processor;
    private final AprilTagClusterMetadata[] clusters = new AprilTagClusterMetadata[4];

    private float decimation;
    private int exposureStep = 0; // 0 = waiting for stream, 1 = mode set, 2 = exposure set, 3 = finished
    private long exposureStepAtNs;

    HiveCamera(HardwareMap hardwareMap) {
        AprilTagLibrary library = AprilTagGameDatabase.getBioBuzzTagLibrary();
        for (AprilTagClusterMetadata cluster : library.getAllClusters()) {
            for (int i = 0; i < CLUSTER_NAMES.length; i++) {
                if (CLUSTER_NAMES[i].equals(cluster.name)) clusters[i] = cluster;
            }
        }

        AprilTagProcessor.Builder tags = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setNumThreads(SensingConstants.DETECTOR_THREADS)
                // Drawing is only ever seen in the live preview
                .setDrawAxes(SensingConstants.ENABLE_LIVE_VIEW)
                .setDrawTagOutline(SensingConstants.ENABLE_LIVE_VIEW)
                .setDrawTagID(SensingConstants.ENABLE_LIVE_VIEW)
                .setDrawCubeProjection(false);
        if (SensingConstants.USE_CUSTOM_LENS_INTRINSICS) {
            tags.setLensIntrinsics(SensingConstants.LENS_FX, SensingConstants.LENS_FY,
                    SensingConstants.LENS_CX, SensingConstants.LENS_CY);
        }
        processor = tags.build();

        decimation = SensingConstants.DECIMATION_FAR;
        processor.setDecimation(decimation);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, SensingConstants.CAMERA_NAME))
                .setCameraResolution(new Size(SensingConstants.CAMERA_WIDTH, SensingConstants.CAMERA_HEIGHT))
                .setStreamFormat(SensingConstants.CAMERA_USE_MJPEG
                        ? VisionPortal.StreamFormat.MJPEG : VisionPortal.StreamFormat.YUY2)
                .enableLiveView(SensingConstants.ENABLE_LIVE_VIEW)
                .setAutoStopLiveView(true)
                .addProcessor(processor)
                .build();
    }

    /**
     * @return the detections from a camera frame that has not been handed out before, or null if
     *         no new frame has arrived since the last call
     */
    ArrayList<AprilTagDetection> poll() {
        if (exposureStep < 3) stepExposure();
        return processor.getFreshDetections();
    }

    /** @return 0-3 for one of the hive clusters (see SensingConstants), or -1 for anything else */
    int clusterIndex(AprilTagClusterMetadata metadata) {
        for (int i = 0; i < 4; i++) {
            if (clusters[i] == metadata) return i;
        }
        if (metadata != null) {
            for (int i = 0; i < 4; i++) {
                if (CLUSTER_NAMES[i].equals(metadata.name)) return i;
            }
        }
        return -1;
    }

    /** Only talks to the detector when the value actually changes. */
    void setDecimation(float value) {
        if (value != decimation) {
            processor.setDecimation(value);
            decimation = value;
        }
    }

    float getDecimation() { return decimation; }
    float getFps() { return portal.getFps(); }
    boolean isStreaming() { return portal.getCameraState() == VisionPortal.CameraState.STREAMING; }
    /** True once the fixed exposure has been applied (or was found not to be supported). */
    boolean isExposureSet() { return exposureStep >= 3; }

    void close() {
        portal.close();
    }

    /**
     * Fixed exposure can only be set once the camera is streaming, and the camera needs a pause
     * between each setting. Rather than sleeping (which would freeze the OpMode), this does one
     * small step per call and comes back later for the next.
     */
    private void stepExposure() {
        if (!SensingConstants.USE_MANUAL_EXPOSURE) {
            exposureStep = 3;
            return;
        }
        if (!isStreaming()) return;

        long now = System.nanoTime();
        if (exposureStep > 0 && now - exposureStepAtNs < EXPOSURE_STEP_GAP_NS) return;

        try {
            if (exposureStep == 0) {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                if (exposure == null || !exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposureStep = 3; // this camera cannot do it; carry on with auto exposure
                    return;
                }
                exposure.setMode(ExposureControl.Mode.Manual);
            } else if (exposureStep == 1) {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                exposure.setExposure(SensingConstants.EXPOSURE_MS, TimeUnit.MILLISECONDS);
            } else {
                GainControl gain = portal.getCameraControl(GainControl.class);
                if (gain != null) {
                    int min = gain.getMinGain();
                    int max = gain.getMaxGain();
                    gain.setGain(min + (int) Math.round((max - min) * SensingConstants.GAIN_FRACTION));
                }
            }
            exposureStep++;
            exposureStepAtNs = now;
        } catch (RuntimeException e) {
            exposureStep = 3; // never let a camera quirk stop the robot
        }
    }
}
