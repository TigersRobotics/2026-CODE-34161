package org.firstinspires.ftc.teamcode.sensing;

import static org.firstinspires.ftc.teamcode.constants.SensingConstants.CLUSTER_WHEN_UP;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.DECIMATION_FAR;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.DECIMATION_NEAR;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.DECIMATION_SWITCH_IN;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.SensingConstants;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * The robot sensing system: where the robot is, and which way each hive is tipped.
 * <p>
 * It combines the goBILDA Pinpoint (odometry pods + gyro), which is smooth but slowly drifts, with
 * a webcam looking at the AprilTags under the hive cells, which is rough but never drifts.
 * <p>
 * HOW TO USE IT (same pattern as the other controllers):
 * <pre>
 *   // init()
 *   RobotSensing.init(hardwareMap);
 *   RobotSensing.setAlliance(RobotSensing.Alliance.RED);
 *
 *   // loop(), after odometry.update()
 *   Pose2D p = odometry.getPose();
 *   RobotSensing.update(p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH),
 *                       p.getHeading(AngleUnit.RADIANS), PollenTurretController.getCurrentYaw());
 *
 *   RobotSensing.getX();  RobotSensing.getY();  RobotSensing.getHeadingRad();   // where am I
 *   RobotSensing.getHiveState(RobotSensing.Alliance.RED);                       // which cell is up
 *   RobotSensing.getTargetYawDeg();  RobotSensing.getTargetDistanceIn();        // where to shoot
 *
 *   // stop()
 *   RobotSensing.stop();
 * </pre>
 * Every number it needs lives in {@link SensingConstants}. If the camera is missing or unplugged
 * the system keeps running on odometry alone instead of crashing the OpMode.
 * <p>
 * update() does no hub or I2C communication of its own and creates no objects, so it adds well
 * under a millisecond to the loop. The tag detection itself runs on the SDK's camera thread.
 */
public class RobotSensing {
    public enum Alliance { RED, BLUE }

    private static final double TARGET_VISIBLE_MS = 250;
    private static final long SEARCH_AFTER_NS = 500_000_000L;

    private static HiveSensingCore core;
    private static HiveCamera camera;
    private static String cameraProblem;
    private static Alliance alliance = Alliance.RED;
    private static long lastHiveSeenNs;
    private static long nowNs;
    private static int frames;

    public static void init(HardwareMap hardwareMap, boolean actuallyEnabled) {
        stop();
        if (!actuallyEnabled) return;

        core = new HiveSensingCore();
        frames = 0;
        lastHiveSeenNs = System.nanoTime();
        nowNs = lastHiveSeenNs;

        try {
            camera = new HiveCamera(hardwareMap);
            cameraProblem = null;
        } catch (RuntimeException e) {
            // Not in the robot configuration, unplugged, or an unsupported resolution
            camera = null;
            cameraProblem = String.valueOf(e.getMessage());
            TimedTelemetry.addLine("SENSING: no camera, running on odometry only", 5000);
        }
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    /**
     * Call once per loop, straight after the odometry has been updated.
     *
     * @param odometryXIn      Pinpoint X in inches
     * @param odometryYIn      Pinpoint Y in inches
     * @param odometryHeadingRad Pinpoint heading in radians
     * @param turretYawDeg     the turret angle as PollenTurretController.getCurrentYaw() reports it
     *                         (ignored if the camera is on the chassis)
     */
    public static void update(double odometryXIn, double odometryYIn, double odometryHeadingRad, double turretYawDeg) {
        if (core == null) return;

        nowNs = System.nanoTime();
        core.addOdometry(nowNs, odometryXIn, odometryYIn, odometryHeadingRad, turretYawDeg);
        if (camera == null) return;

        ArrayList<AprilTagDetection> detections = camera.poll();
        if (detections == null) return; // no new camera frame this loop, which is the usual case

        frames++;
        core.beginFrame();
        for (int i = 0, n = detections.size(); i < n; i++) {
            AprilTagDetection detection = detections.get(i);
            if (!(detection instanceof AprilTagClusterDetection)) continue;

            AprilTagClusterDetection hive = (AprilTagClusterDetection) detection;
            int cluster = camera.clusterIndex(hive.metadata);
            if (cluster < 0 || hive.ftcPose == null || hive.rawPose == null) continue;

            // The third column of the rotation is the direction the sticker faces
            MatrixF rotation = hive.rawPose.R;
            core.addDetection(nowNs, cluster, hive.ftcPose.x, hive.ftcPose.y, hive.ftcPose.z,
                    rotation.get(0, 2), rotation.get(1, 2), rotation.get(2, 2),
                    hive.percentClusterFound, hive.frameAcquisitionNanoTime);
        }

        // Close to the hive the tags are big, so the detector can run on a smaller picture and go
        // faster. Far away, or while searching, it needs the detail.
        double nearest = core.getFrameNearestRange();
        if (nearest != Double.POSITIVE_INFINITY) {
            lastHiveSeenNs = nowNs;
            if (nearest < DECIMATION_SWITCH_IN - 5) camera.setDecimation(DECIMATION_NEAR);
            else if (nearest > DECIMATION_SWITCH_IN + 5) camera.setDecimation(DECIMATION_FAR);
        } else if (nowNs - lastHiveSeenNs > SEARCH_AFTER_NS) {
            camera.setDecimation(DECIMATION_FAR);
        }
    }

    public static void stop() {
        if (camera != null) camera.close();
        camera = null;
        core = null;
    }

    /** Call if the odometry is given a new pose mid-OpMode, so old corrections are not applied to it. */
    public static void onOdometryReset() {
        if (core != null) core.resetPose();
    }

    public static void setAlliance(Alliance alliance) {
        RobotSensing.alliance = alliance;
    }

    public static Alliance getAlliance() { return alliance; }

    // =============================================================================================
    // WHERE THE ROBOT IS
    // =============================================================================================

    /** True once at least one odometry reading has been given to update(). */
    public static boolean hasPose() { return core != null && core.fusion.hasPose(); }

    /** Field X in inches. */
    public static double getX() { return core == null ? Double.NaN : core.fusion.getX(); }
    /** Field Y in inches. */
    public static double getY() { return core == null ? Double.NaN : core.fusion.getY(); }
    /** Heading in radians, counter-clockwise, between -pi and pi. */
    public static double getHeadingRad() { return core == null ? Double.NaN : core.fusion.getHeading(); }
    public static double getHeadingDeg() { return Math.toDegrees(getHeadingRad()); }

    /** The same pose as an SDK object. This one creates an object, so prefer getX()/getY() inside the loop. */
    public static Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, getX(), getY(), AngleUnit.RADIANS, getHeadingRad());
    }

    /** Field velocity in inches per second. */
    public static double getVelocityX() { return core == null ? 0 : core.fusion.getVelX(); }
    public static double getVelocityY() { return core == null ? 0 : core.fusion.getVelY(); }

    /** How sure the system is of the position: roughly "within this many inches". */
    public static double getPositionSigmaIn() { return core == null ? Double.NaN : core.fusion.getSigmaIn(); }

    /** How far the camera has moved the position away from raw odometry, in inches. */
    public static double getCorrectionIn() {
        return core == null ? 0 : Math.hypot(core.fusion.getOffsetX(), core.fusion.getOffsetY());
    }

    /** Milliseconds since the camera last corrected the position (infinity if it never has). */
    public static double getMsSinceCameraFix() {
        return core == null ? Double.POSITIVE_INFINITY : core.fusion.msSinceVisionFix(nowNs);
    }

    // =============================================================================================
    // THE HIVES
    // =============================================================================================

    /** Which cell of that alliance's hive is pointing up, or UNKNOWN if the camera has not seen it yet. */
    public static HiveTracker.State getHiveState(Alliance hive) {
        return core == null ? HiveTracker.State.UNKNOWN : core.hives.getState(hiveIndex(hive));
    }

    /** How many times that hive has been seen to tip since init. */
    public static int getTipCount(Alliance hive) {
        return core == null ? 0 : core.hives.getTipCount(hiveIndex(hive));
    }

    /** Milliseconds since that hive last tipped (infinity if it has not). */
    public static double getMsSinceTip(Alliance hive) {
        return core == null ? Double.POSITIVE_INFINITY : core.hives.msSinceTip(hiveIndex(hive), nowNs);
    }

    /** True for a short time after that hive tips. Handy for "stop shooting, it has gone over". */
    public static boolean hasJustTipped(Alliance hive, double withinMs) {
        return getMsSinceTip(hive) <= withinMs;
    }

    /** Milliseconds since the camera last had a clear look at that hive. The state is remembered in between. */
    public static double getMsSinceHiveSeen(Alliance hive) {
        return core == null ? Double.POSITIVE_INFINITY : core.hives.msSinceSeen(hiveIndex(hive), nowNs);
    }

    /** 0 = no idea which way that hive is tipped, 1 = certain. */
    public static double getHiveConfidence(Alliance hive) {
        return core == null ? 0 : core.hives.getConfidence(hiveIndex(hive));
    }

    // =============================================================================================
    // WHERE TO SHOOT (the upward cell of our own alliance's hive)
    // =============================================================================================

    /** True if we know which cell to shoot at and its position has been measured. */
    public static boolean hasTarget() { return !Double.isNaN(getTargetX()); }

    /** Field X of the cell to shoot at, in inches. NaN until the hive state is known and calibrated. */
    public static double getTargetX() {
        int cluster = targetCluster();
        return cluster < 0 ? Double.NaN : CLUSTER_WHEN_UP[cluster][0];
    }

    /** Field Y of the cell to shoot at, in inches. */
    public static double getTargetY() {
        int cluster = targetCluster();
        return cluster < 0 ? Double.NaN : CLUSTER_WHEN_UP[cluster][1];
    }

    /** Height above the tiles of the tag reference point of the cell to shoot at, in inches. */
    public static double getTargetHeightIn() {
        int cluster = targetCluster();
        return cluster < 0 ? Double.NaN : CLUSTER_WHEN_UP[cluster][2];
    }

    /** Distance along the floor from the robot to the cell to shoot at, in inches. */
    public static double getTargetDistanceIn() {
        return Math.hypot(getTargetX() - getX(), getTargetY() - getY());
    }

    /**
     * The angle to the cell to shoot at, measured from the front of the robot, in degrees,
     * COUNTER-CLOCKWISE positive, between -180 and 180. PollenTurretController counts clockwise as
     * positive, so negate this before passing it to setYaw().
     */
    public static double getTargetYawDeg() {
        double fieldAngle = Math.atan2(getTargetY() - getY(), getTargetX() - getX());
        return Math.toDegrees(PoseFusion.wrap(fieldAngle - getHeadingRad()));
    }

    /** True if the camera can see the tags of the cell to shoot at right now. */
    public static boolean isTargetVisible() {
        int cluster = targetCluster();
        return cluster >= 0 && core.isSeenWithin(cluster, nowNs, TARGET_VISIBLE_MS);
    }

    /**
     * How far the turret is from pointing straight at the target cell's tags, measured directly by
     * the camera, in degrees, counter-clockwise positive. It does not depend on the position
     * estimate at all, so it can be used to fine-trim the aim. NaN when the target is not in view.
     */
    public static double getCameraAimErrorDeg() {
        int cluster = targetCluster();
        if (cluster < 0 || !core.isSeenWithin(cluster, nowNs, TARGET_VISIBLE_MS)) return Double.NaN;
        return Math.toDegrees(core.getMeasuredAimRad(cluster));
    }

    // =============================================================================================
    // FOR TESTING AND CALIBRATION
    // =============================================================================================

    public static boolean isCameraWorking() { return camera != null; }
    public static HiveSensingCore getCore() { return core; }

    /** Adds a block of status lines. Only for test OpModes: building the text creates garbage. */
    public static void addTelemetry(Telemetry telemetry) {
        if (core == null) {
            telemetry.addLine("SENSING: not initialised");
            return;
        }
        PoseFusion fusion = core.fusion;
        telemetry.addData("Pose (in, deg)", "%.1f, %.1f, %.1f", getX(), getY(), getHeadingDeg());
        telemetry.addData("Sure within", "%.1f in", getPositionSigmaIn());
        telemetry.addData("Camera correction", "%.1f in (x %.1f, y %.1f)", getCorrectionIn(), fusion.getOffsetX(), fusion.getOffsetY());
        telemetry.addData("Camera fixes", "%d used, %d rejected, %d catch-ups, %d re-locks",
                fusion.getAcceptedCount(), fusion.getRejectedCount(), fusion.getCatchUpCount(), fusion.getRelockCount());
        telemetry.addData("Last fix", "%.0f ms ago, %.1f in from fused", getMsSinceCameraFix(), fusion.getLastInnovationIn());

        if (camera == null) {
            telemetry.addData("Camera", "NOT WORKING: %s", cameraProblem);
        } else {
            telemetry.addData("Camera", "%.0f fps, %.0f ms late, decimation %.0f, exposure %s, %d frames",
                    camera.getFps(), core.getLastLatencyMs(), camera.getDecimation(),
                    camera.isExposureSet() ? "set" : "pending", frames);
            if (core.getTimestampFallbacks() > 0) {
                telemetry.addData("Bad frame timestamps", core.getTimestampFallbacks());
            }
        }

        telemetry.addData("RED hive", "%s (sure %.0f%%, %d tips)", getHiveState(Alliance.RED), getHiveConfidence(Alliance.RED) * 100, getTipCount(Alliance.RED));
        telemetry.addData("BLUE hive", "%s (sure %.0f%%, %d tips)", getHiveState(Alliance.BLUE), getHiveConfidence(Alliance.BLUE) * 100, getTipCount(Alliance.BLUE));
        telemetry.addData("Target (" + alliance + ")", hasTarget()
                ? String.format("%.1f in away, yaw %.1f deg, camera aim error %.1f deg", getTargetDistanceIn(), getTargetYawDeg(), getCameraAimErrorDeg())
                : "not known yet");
    }

    private static int hiveIndex(Alliance hive) {
        return hive == Alliance.RED ? HiveTracker.RED_HIVE : HiveTracker.BLUE_HIVE;
    }

    private static int targetCluster() {
        return core == null ? -1 : core.getUpCluster(hiveIndex(alliance));
    }
}
