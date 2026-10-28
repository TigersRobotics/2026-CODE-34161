package org.firstinspires.ftc.teamcode.sensors;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagWebcam {
    //Processor for the april tag
    private AprilTagProcessor processor;

    //Vison/ camera I think
    private VisionPortal vision;

    //List of Detected Tags
    private ArrayList<AprilTagDetection> detectedTags = new ArrayList<>();

    public AprilTagWebcam(HardwareMap hardwareMap) {
        AprilTagProcessor.Builder processerBuilder = new AprilTagProcessor.Builder();
        processerBuilder.setDrawTagID(true);
        processerBuilder.setDrawTagOutline(true);
        processerBuilder.setDrawAxes(true);
        processerBuilder.setDrawCubeProjection(true);
        processerBuilder.setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES);

        processor = processerBuilder.build();

        VisionPortal.Builder visionBuilder = new VisionPortal.Builder();
        visionBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        visionBuilder.addProcessor(processor);
        visionBuilder.enableLiveView(true);

        vision = visionBuilder.build();
    }
    public void updateTagDetection() {
        if(processor.getDetections() != null) {
            detectedTags = processor.getDetections();
        }
    }

    public ArrayList<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }
    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("# AprilTags Detected", detectedTags.size());

        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    //Turn off vision to save resources
    public void stop() {
        //Detect if there is a vision portal open
        if(vision != null) {
            vision.close();
        }
    }

    public AprilTagDetection getTagByID(int id) {
        for(AprilTagDetection detection : detectedTags) {
            AprilTagSingleDetection singleTag = (AprilTagSingleDetection) detection;
            if(singleTag.id == id) {
                return detection;
            }
        }
        return null;
    }
}
