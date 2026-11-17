package org.firstinspires.ftc.teamcode.sensing;

import static org.firstinspires.ftc.teamcode.constants.SensingConstants.*;

/**
 * The maths of the sensing system: turns "the camera saw cluster N over there" into a hive state
 * and a position fix.
 * <p>
 * For every cluster the camera sees, in this order:
 * <ol>
 * <li>Look up where the odometry and turret were when the picture was taken.</li>
 * <li>Swing the camera's measurement through the camera mount, the turret and the robot heading,
 *     giving the offset from the robot to the cluster in field directions, and its height.</li>
 * <li>Decide whether that cell is UP or DOWN. This uses the HEIGHT of the tags (and optionally
 *     which way the sticker leans). Neither depends on knowing where the robot is, so the tip
 *     state can be trusted even when the position is off.</li>
 * <li>Only if the cell is clearly in one of its two resting places: subtract the offset from that
 *     known place to get where the robot must be, and hand that to {@link PoseFusion}.</li>
 * </ol>
 * The robot's heading is taken from the gyro rather than from the tags. A webcam is good at
 * "how far and in which direction", and bad at "what angle is that sticker at", so only the good
 * part is used for position.
 * <p>
 * No FTC SDK in here and nothing is allocated per loop, so this can be run and tested on a laptop.
 */
public final class HiveSensingCore {
    private static final int[] HIVE_OF_CLUSTER = { HiveTracker.RED_HIVE, HiveTracker.RED_HIVE, HiveTracker.BLUE_HIVE, HiveTracker.BLUE_HIVE };
    private static final boolean[] IS_AUDIENCE_CELL = { false, true, true, false };

    private static final long MAX_FRAME_AGE_NS = (long) (MAX_FRAME_AGE_MS * 1e6);
    private static final long FALLBACK_LATENCY_NS = (long) (FALLBACK_LATENCY_MS * 1e6);
    private static final double MAX_TURN_RAD_PER_S = Math.toRadians(VISION_MAX_TURN_DEG_PER_S);
    private static final double HALF_TRUST_TURN_RAD_PER_S = Math.toRadians(VISION_TURN_FOR_HALF_TRUST_DEG_PER_S);
    private static final double TURRET_DEG_TO_RAD = Math.toRadians(TURRET_YAW_SIGN);
    private static final double TURRET_ZERO_RAD = Math.toRadians(TURRET_ZERO_OFFSET_DEG);

    // The camera mount never changes, so its sines and cosines are worked out once
    private static final double COS_ROLL = Math.cos(Math.toRadians(CAMERA_ROLL_DEG));
    private static final double SIN_ROLL = Math.sin(Math.toRadians(CAMERA_ROLL_DEG));
    private static final double COS_PITCH = Math.cos(Math.toRadians(CAMERA_PITCH_UP_DEG));
    private static final double SIN_PITCH = Math.sin(Math.toRadians(CAMERA_PITCH_UP_DEG));
    private static final double COS_YAW = Math.cos(Math.toRadians(CAMERA_YAW_LEFT_DEG));
    private static final double SIN_YAW = Math.sin(Math.toRadians(CAMERA_YAW_LEFT_DEG));

    public final PoseFusion fusion = new PoseFusion(START_POSITION_SIGMA_IN);
    public final HiveTracker hives = new HiveTracker();

    // The latest reading of each cluster (index = cluster number from SensingConstants)
    private final long[] seenNs = { Long.MIN_VALUE, Long.MIN_VALUE, Long.MIN_VALUE, Long.MIN_VALUE };
    private final double[] measX = new double[4];
    private final double[] measY = new double[4];
    private final double[] measZ = new double[4];
    private final double[] measCue = new double[4];
    private final double[] measRange = new double[4];
    private final double[] measScore = new double[4];
    private final double[] measAimRad = new double[4];
    private final int[] measPercent = new int[4];

    private double frameNearestRange = Double.POSITIVE_INFINITY;
    private double lastLatencyMs = Double.NaN;
    private int timestampFallbacks;

    // Scratch outputs of toMountFrame(), reused so nothing is allocated
    private double vForward, vLeft, vUp;

    public HiveSensingCore() {
        hives.reset(RED_HIVE_START, BLUE_HIVE_START);
    }

    /** Call when the odometry has been given a new pose, so old corrections are not applied to it. */
    public void resetPose() {
        fusion.reset(START_POSITION_SIGMA_IN);
    }

    /**
     * Call once per loop, before any detections.
     *
     * @param turretYawDeg the turret angle exactly as the turret controller reports it
     */
    public void addOdometry(long nowNs, double x, double y, double headingRad, double turretYawDeg) {
        double turretRad = CAMERA_ON_TURRET ? turretYawDeg * TURRET_DEG_TO_RAD + TURRET_ZERO_RAD : 0;
        fusion.addOdometry(nowNs, x, y, headingRad, turretRad);
    }

    /** Call once before the detections of each new camera frame. */
    public void beginFrame() {
        frameNearestRange = Double.POSITIVE_INFINITY;
    }

    /**
     * One cluster seen in one camera frame.
     *
     * @param cluster      0-3, see SensingConstants
     * @param camRight     cluster position relative to the camera: inches to the right
     * @param camForward   inches out along the lens axis
     * @param camUp        inches up
     * @param normalX      the direction the sticker faces, in OpenCV camera axes (x right, y down, z out)
     * @param percentFound how many of the cluster's tags were found, 25-100
     * @param frameNs      when the picture was taken (System.nanoTime() clock)
     */
    public void addDetection(long nowNs, int cluster, double camRight, double camForward, double camUp,
                             double normalX, double normalY, double normalZ, int percentFound, long frameNs) {
        // 1. Where was the robot when this picture was taken?
        long age = nowNs - frameNs;
        if (age < 0 || age > MAX_FRAME_AGE_NS) {
            frameNs = nowNs - FALLBACK_LATENCY_NS;
            timestampFallbacks++;
        }
        lastLatencyMs = (nowNs - frameNs) * 1e-6;
        if (!fusion.sample(frameNs)) return;

        double cosTurret = Math.cos(fusion.sampleTurret);
        double sinTurret = Math.sin(fusion.sampleTurret);
        double cosHeading = Math.cos(fusion.sampleHeading);
        double sinHeading = Math.sin(fusion.sampleHeading);

        // 2. Camera -> turret -> robot -> field
        toMountFrame(camRight, camForward, camUp);
        double turretForward = vForward + CAMERA_FORWARD_OF_TURRET_AXIS_IN;
        double turretLeft = vLeft + CAMERA_LEFT_OF_TURRET_AXIS_IN;
        double height = vUp + CAMERA_HEIGHT_IN;

        double robotForward = turretForward * cosTurret - turretLeft * sinTurret + TURRET_AXIS_FORWARD_IN;
        double robotLeft = turretForward * sinTurret + turretLeft * cosTurret + TURRET_AXIS_LEFT_IN;
        double fieldDx = robotForward * cosHeading - robotLeft * sinHeading;
        double fieldDy = robotForward * sinHeading + robotLeft * cosHeading;

        // The way the sticker faces, turned the same way (a direction, so no offsets are added)
        toMountFrame(normalX, normalZ, -normalY);
        double normalRobotForward = vForward * cosTurret - vLeft * sinTurret;
        double normalRobotLeft = vForward * sinTurret + vLeft * cosTurret;
        double tiltCue = (normalRobotForward * cosHeading - normalRobotLeft * sinHeading) * TIP_AXIS_X
                + (normalRobotForward * sinHeading + normalRobotLeft * cosHeading) * TIP_AXIS_Y;

        double range = Math.sqrt(camRight * camRight + camForward * camForward + camUp * camUp);
        if (range < frameNearestRange) frameNearestRange = range;

        // 3. Is this cell UP or DOWN?  +1 = exactly the UP measurement, -1 = exactly DOWN
        double[] whenUp = CLUSTER_WHEN_UP[cluster];
        double[] whenDown = CLUSTER_WHEN_DOWN[cluster];
        double heightScore = cueScore(height, whenUp[2], whenDown[2], 1.0);
        double tiltScore = cueScore(tiltCue, whenUp[3], whenDown[3], 0.1);
        double score = combine(heightScore, tiltScore);

        seenNs[cluster] = nowNs;
        measX[cluster] = fusion.sampleX + fusion.getOffsetX() + fieldDx;
        measY[cluster] = fusion.sampleY + fusion.getOffsetY() + fieldDy;
        measZ[cluster] = height;
        measCue[cluster] = tiltCue;
        measRange[cluster] = range;
        measScore[cluster] = score;
        measAimRad[cluster] = Math.atan2(turretLeft, turretForward);
        measPercent[cluster] = percentFound;

        // NaN (not calibrated, or the two cues contradict each other) fails this test too
        if (!(Math.abs(score) >= TIP_MIN_FRAME_SCORE)) return;
        if (percentFound < VISION_MIN_CLUSTER_PERCENT || range > VISION_MAX_RANGE_IN) return;

        boolean cellIsUp = score > 0;
        int hive = HIVE_OF_CLUSTER[cluster];
        hives.vote(hive, cellIsUp == IS_AUDIENCE_CELL[cluster], nowNs);

        // 4. A position fix, but only from a cell that is resting where the hive state says it is.
        //    While a hive is swinging over, its tags are somewhere in between and would mislead us.
        HiveTracker.State state = hives.getState(hive);
        if (state == HiveTracker.State.UNKNOWN) return;
        boolean stateSaysUp = (state == HiveTracker.State.AUDIENCE_CELL_UP) == IS_AUDIENCE_CELL[cluster];
        if (stateSaysUp != cellIsUp) return;

        double[] place = cellIsUp ? whenUp : whenDown;
        if (Double.isNaN(place[0]) || Double.isNaN(place[1])) return;

        double turning = Math.abs(fusion.getTurnRate() + fusion.getTurretRate());
        if (turning > MAX_TURN_RAD_PER_S) return;

        // How much to trust this fix: worse when far away, moving, turning, or missing tags
        double sigma = VISION_SIGMA_BASE_IN + VISION_SIGMA_PER_RANGE_SQUARED * range * range;
        sigma *= 1.0 + fusion.getSpeed() / VISION_SPEED_FOR_HALF_TRUST_IN_PER_S;
        sigma *= 1.0 + turning / HALF_TRUST_TURN_RAD_PER_S;
        sigma *= 1.0 + (100 - percentFound) * 0.01;

        fusion.addVision(frameNs, place[0] - fieldDx, place[1] - fieldDy, sigma * sigma, nowNs);
    }

    /**
     * Undoes the camera's roll, pitch and yaw. Input is (right, forward, up) as the camera sees it;
     * the result, in vForward / vLeft / vUp, is level and lined up with the turret.
     */
    private void toMountFrame(double right, double forward, double up) {
        double levelRight = right * COS_ROLL - up * SIN_ROLL;
        double rolledUp = right * SIN_ROLL + up * COS_ROLL;
        double levelForward = forward * COS_PITCH - rolledUp * SIN_PITCH;
        double left = -levelRight;
        vUp = forward * SIN_PITCH + rolledUp * COS_PITCH;
        vForward = levelForward * COS_YAW - left * SIN_YAW;
        vLeft = levelForward * SIN_YAW + left * COS_YAW;
    }

    /** Where a reading sits between the DOWN measurement (-1) and the UP measurement (+1). NaN if not calibrated. */
    private static double cueScore(double value, double whenUp, double whenDown, double minSeparation) {
        double half = (whenUp - whenDown) * 0.5;
        if (!(Math.abs(half) * 2 >= minSeparation)) return Double.NaN; // also catches NaN
        return (value - (whenUp + whenDown) * 0.5) / half;
    }

    /** Blends the two cues. Returns NaN when there is nothing usable, or the reading makes no sense. */
    private static double combine(double heightScore, double tiltScore) {
        boolean hasHeight = !Double.isNaN(heightScore);
        boolean hasTilt = !Double.isNaN(tiltScore);
        if (hasHeight && Math.abs(heightScore) > TIP_MAX_FRAME_SCORE) return Double.NaN;
        if (hasTilt && Math.abs(tiltScore) > TIP_MAX_FRAME_SCORE) hasTilt = false; // tilt is the flakier cue

        if (hasHeight && hasTilt) {
            boolean bothSure = Math.abs(heightScore) >= TIP_MIN_FRAME_SCORE && Math.abs(tiltScore) >= TIP_MIN_FRAME_SCORE;
            if (bothSure && (heightScore > 0) != (tiltScore > 0)) return Double.NaN; // they contradict each other
            return (heightScore * HEIGHT_CUE_WEIGHT + tiltScore * TILT_CUE_WEIGHT) / (HEIGHT_CUE_WEIGHT + TILT_CUE_WEIGHT);
        }
        if (hasHeight) return heightScore;
        if (hasTilt) return tiltScore;
        return Double.NaN;
    }

    // ---- results -------------------------------------------------------------------------------

    /** @return the cluster (0-3) under the cell of this hive that is pointing up, or -1 if not known */
    public int getUpCluster(int hive) {
        HiveTracker.State state = hives.getState(hive);
        if (state == HiveTracker.State.UNKNOWN) return -1;
        boolean audience = state == HiveTracker.State.AUDIENCE_CELL_UP;
        if (hive == HiveTracker.RED_HIVE) return audience ? RED_AUDIENCE : RED_SCORING;
        return audience ? BLUE_AUDIENCE : BLUE_SCORING;
    }

    public boolean isSeenWithin(int cluster, long nowNs, double ms) {
        return seenNs[cluster] != Long.MIN_VALUE && (nowNs - seenNs[cluster]) * 1e-6 <= ms;
    }
    /** System.nanoTime() of the loop this cluster was last seen in, or Long.MIN_VALUE if never. */
    public long getSeenNs(int cluster) { return seenNs[cluster]; }
    /** Field X of where the camera last put this cluster (uses the current position estimate). */
    public double getMeasuredX(int cluster) { return measX[cluster]; }
    public double getMeasuredY(int cluster) { return measY[cluster]; }
    /** Height above the tiles of where the camera last put this cluster. Does not depend on the position estimate. */
    public double getMeasuredHeight(int cluster) { return measZ[cluster]; }
    public double getMeasuredTiltCue(int cluster) { return measCue[cluster]; }
    public double getMeasuredRange(int cluster) { return measRange[cluster]; }
    /** -1 = looked exactly DOWN, +1 = exactly UP, NaN = not calibrated or unusable. */
    public double getMeasuredScore(int cluster) { return measScore[cluster]; }
    public int getMeasuredPercent(int cluster) { return measPercent[cluster]; }
    /** Angle, counter-clockwise in radians, from where the turret points to this cluster. */
    public double getMeasuredAimRad(int cluster) { return measAimRad[cluster]; }

    /** Distance to the nearest cluster in the newest camera frame, or infinity if it had none. */
    public double getFrameNearestRange() { return frameNearestRange; }
    public double getLastLatencyMs() { return lastLatencyMs; }
    public int getTimestampFallbacks() { return timestampFallbacks; }
}
