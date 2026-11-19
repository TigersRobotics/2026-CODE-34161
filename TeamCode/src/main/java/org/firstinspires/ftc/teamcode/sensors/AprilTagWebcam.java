package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;

public class AprilTagWebcam {
    //Processor for the april tag
    private static AprilTagProcessor processor;

    //Vison/ camera I think
    private static VisionPortal vision;

    //List of Detected Tags
    private static ArrayList<AprilTagDetection> detectedTags = new ArrayList<>();
    private static HiveSide lastSeenSide = HiveSide.RIGHT;

    public static void init(HardwareMap hardwareMap) {
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
    public static void update() {
        if(processor.getDetections() != null) {
            detectedTags = processor.getDetections();
        }
    }

    public ArrayList<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }
    public static void displayTelemetry(Telemetry telemetry) {
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
    public static void stop() {
        //Detect if there is a vision portal open
        if(vision != null) {
            vision.close();
            processor = null;
            vision = null;
        }
    }


    public enum HiveSide  {
        LEFT,
        RIGHT
    }

    /**
     * Returns the side hive is on; if there r no detected tags returns the last side seen
     * @param tags list of tags on the hive
     * @param headingRad heading of robot in deg
     * @return enum of hive side
     */
    public static HiveSide getHiveSide(int[] tags, double headingRad) {
        AprilTagDetection best = null;
        for (int id : tags) {
            AprilTagDetection tag = getTagByID(id);
            if (tag == null || tag.ftcPose == null) continue;
            if (best == null || Math.abs(tag.ftcPose.bearing) < Math.abs(best.ftcPose.bearing)) {
                best = tag;
            }
        }
        if (best == null) return lastSeenSide;

        double bearingRad = Math.toRadians(best.ftcPose.bearing);
        double fieldBearing = AngleUnit.normalizeRadians(headingRad + bearingRad);

        // Decide using fieldBearing's sign / range, per your field's convention
        HiveSide side = fieldBearing > 0 ? HiveSide.LEFT : HiveSide.RIGHT;
        lastSeenSide = side;
        return side;
    }
    public static AprilTagDetection getTagByID(int id) {
        for(AprilTagDetection detection : detectedTags) {
            AprilTagSingleDetection singleTag = (AprilTagSingleDetection) detection;
            if(singleTag.id == id) {
                return detection;
            }
        }
        return null;
    }
}
