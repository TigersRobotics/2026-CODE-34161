package org.firstinspires.ftc.teamcode.constants;

/**
 * Every number the robot sensing system needs.
 * <p>
 * HOW TO USE THIS FILE
 * <ul>
 * <li><b>INPUT NEEDED</b> = a value that has to be measured on the real robot or field.
 *     Until it is filled in, the code plays safe: the robot still knows where it is from
 *     odometry, it just does not use the camera for that part yet.</li>
 * <li><b>TUNE</b> = a sensible starting value that can be changed after watching the
 *     "Sensing Test" OpMode.</li>
 * <li>Anything set to {@code Double.NaN} means "not measured yet".</li>
 * </ul>
 * UNITS: inches and degrees, in the SAME field coordinates the Pinpoint odometry uses
 * (the ones the start pose is set in). Heading is counter-clockwise positive.
 * Robot axes: X is forward, Y is to the robot's left.
 */
public final class SensingConstants {
    private SensingConstants() {}

    // =============================================================================================
    // 1. CAMERA HARDWARE
    // =============================================================================================

    /** INPUT NEEDED: the camera's name in the robot configuration on the Driver Station. */
    public static final String CAMERA_NAME = "camera";

    /**
     * INPUT NEEDED: resolution to run the camera at. 1280x800 is the native size of the
     * recommended Arducam OV9281 global-shutter camera. For a Logitech C920/C270 use 640x480.
     */
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 800;

    /** MJPEG gives far more frames per second than YUY2 at this resolution. Set false if the camera has no MJPEG mode. */
    public static final boolean CAMERA_USE_MJPEG = true;

    /**
     * INPUT NEEDED: lens calibration. The OV9281 is not in the SDK's built-in calibration list, so
     * these four numbers must come from calibrating YOUR camera at the resolution above
     * (ftc-docs "Camera Calibration", or calibdb.net). The values below are only a rough guess
     * for a ~70 degree lens, and every distance the camera reports is wrong until they are replaced.
     * <p>
     * For a Logitech C920/C270 at 640x480 set USE_CUSTOM_LENS_INTRINSICS to false and the SDK's
     * own calibration is used instead.
     */
    public static final boolean USE_CUSTOM_LENS_INTRINSICS = true;
    public static final double LENS_FX = 915.0;
    public static final double LENS_FY = 915.0;
    public static final double LENS_CX = 640.0;
    public static final double LENS_CY = 400.0;

    /**
     * TUNE: a short fixed exposure stops the tags smearing while the robot or turret moves, which
     * matters more than a bright picture. If tags stop being found, raise the exposure a little
     * before raising the gain.
     */
    public static final boolean USE_MANUAL_EXPOSURE = true;
    public static final long EXPOSURE_MS = 3;
    /** TUNE: 0 = lowest gain the camera has, 1 = highest. */
    public static final double GAIN_FRACTION = 0.7;

    /** The live preview costs CPU on the Control Hub. Turn it on only while aiming the camera. */
    public static final boolean ENABLE_LIVE_VIEW = false;

    /** TUNE: threads the tag detector may use. The Control Hub has 4 cores; 3 leaves one for the OpMode. */
    public static final int DETECTOR_THREADS = 3;

    /**
     * TUNE: decimation shrinks the picture before searching for tags. Higher = faster but shorter
     * range. The system uses the NEAR value while the hive is closer than DECIMATION_SWITCH_IN
     * and the FAR value otherwise (and while it is searching).
     */
    public static final float DECIMATION_NEAR = 3f;
    public static final float DECIMATION_FAR = 2f;
    public static final double DECIMATION_SWITCH_IN = 50.0;

    // =============================================================================================
    // 2. WHERE THE CAMERA IS ON THE ROBOT
    // =============================================================================================

    /** true = the camera rides on the turret. false = it is bolted to the chassis. */
    public static final boolean CAMERA_ON_TURRET = true;

    /**
     * INPUT NEEDED: where the turret's rotation axis is on the robot, measured from the point the
     * Pinpoint tracks (the centre the odometry offsets are measured from).
     * If the camera is on the chassis, put the camera lens position here instead.
     */
    public static final double TURRET_AXIS_FORWARD_IN = 0.0;
    public static final double TURRET_AXIS_LEFT_IN = 0.0;

    /**
     * INPUT NEEDED: where the camera lens is, measured from the turret's rotation axis with the
     * turret pointing straight ahead. Leave both at 0 if the camera is on the chassis.
     */
    public static final double CAMERA_FORWARD_OF_TURRET_AXIS_IN = 0.0;
    public static final double CAMERA_LEFT_OF_TURRET_AXIS_IN = 0.0;

    /** INPUT NEEDED: height of the centre of the camera lens above the field tiles. */
    public static final double CAMERA_HEIGHT_IN = 12.0;

    /**
     * INPUT NEEDED: how far the camera is tilted UP from level. The tags are on the underside of
     * the cells, so the camera has to look upward. An error of 1 degree here is roughly
     * 1 inch of height error at 5 feet, so measure it carefully (a phone level app works).
     */
    public static final double CAMERA_PITCH_UP_DEG = 30.0;

    /** INPUT NEEDED: camera pointing left (+) or right (-) of the turret's shooting direction. Usually 0. */
    public static final double CAMERA_YAW_LEFT_DEG = 0.0;

    /** INPUT NEEDED: camera twisted about its own lens axis. Usually 0. */
    public static final double CAMERA_ROLL_DEG = 0.0;

    /**
     * INPUT NEEDED: turns the turret angle the code is given into "counter-clockwise from robot
     * forward". PollenTurretController counts clockwise as positive, so the sign is -1.
     * The offset is the turret angle that points straight ahead (0 if it is zeroed facing forward).
     */
    public static final double TURRET_YAW_SIGN = -1.0;
    public static final double TURRET_ZERO_OFFSET_DEG = 0.0;

    // =============================================================================================
    // 3. THE HIVE (FIELD VALUES)
    // =============================================================================================
    //
    // Each hive is a see-saw with a cell on each end, and each cell has a cluster of 4 AprilTags
    // on its underside. When the hive tips, the tags MOVE. So every cluster has two possible
    // places it can be: its cell UP, or its cell DOWN.
    //
    // INPUT NEEDED: the 8 rows below. Run the "Hive Tag Calibration" OpMode, which prints these
    // numbers for whichever clusters the camera can see. Do it once with each hive tipped each way.
    //
    // Each row is { field X, field Y, height above tiles, tilt cue }.
    //   - X, Y, height: where the SDK's reference point for that cluster is.
    //   - tilt cue: a number the calibration OpMode prints. It measures which way the tag sticker
    //     is leaning, and is a second way of telling UP from DOWN. Leave as NaN to ignore it.
    //
    // Cluster order (tag IDs):  0 = RED SCORING (30-33)    1 = RED AUDIENCE (34-37)
    //                           2 = BLUE AUDIENCE (38-41)  3 = BLUE SCORING (42-45)
    // "AUDIENCE" is the cell on the audience side of the field, "SCORING" is the far side.

    public static final int RED_SCORING = 0;
    public static final int RED_AUDIENCE = 1;
    public static final int BLUE_AUDIENCE = 2;
    public static final int BLUE_SCORING = 3;

    private static final double N = Double.NaN;

    /** INPUT NEEDED: each cluster while its cell is the one pointing UP. */
    public static final double[][] CLUSTER_WHEN_UP = {
            { N, N, N, N },   // 0 RED SCORING
            { N, N, N, N },   // 1 RED AUDIENCE
            { N, N, N, N },   // 2 BLUE AUDIENCE
            { N, N, N, N },   // 3 BLUE SCORING
    };

    /** INPUT NEEDED: each cluster while its cell is the one pointing DOWN. */
    public static final double[][] CLUSTER_WHEN_DOWN = {
            { N, N, N, N },   // 0 RED SCORING
            { N, N, N, N },   // 1 RED AUDIENCE
            { N, N, N, N },   // 2 BLUE AUDIENCE
            { N, N, N, N },   // 3 BLUE SCORING
    };

    /**
     * INPUT NEEDED: the direction the hives rock in, as a field direction. The cells are on the
     * audience side and the far side, so this is the field axis that runs from the audience
     * towards the far wall. (0, 1) means "along field +Y". Use (1, 0) if that is +X in your
     * coordinates. It only affects the tilt cue.
     */
    public static final double TIP_AXIS_X = 0.0;
    public static final double TIP_AXIS_Y = 1.0;

    /**
     * INPUT NEEDED (optional): which cell of each hive is UP when the match starts.
     * 0 = not known (the system waits until the camera sees the hive),
     * +1 = the AUDIENCE-side cell is up, -1 = the SCORING-side (far) cell is up.
     * The manual stages the hives so that the cell pointing at a FLOWER starts DOWN.
     */
    public static final int RED_HIVE_START = 0;
    public static final int BLUE_HIVE_START = 0;

    // =============================================================================================
    // 4. TELLING WHETHER A HIVE IS TIPPED
    // =============================================================================================

    /** TUNE: how much the height of the tags counts versus the tilt of the tags. */
    public static final double HEIGHT_CUE_WEIGHT = 1.0;
    public static final double TILT_CUE_WEIGHT = 1.0;

    /**
     * TUNE: a camera frame only counts as a vote if it lands at least this far towards UP or DOWN.
     * (-1 = exactly the DOWN measurement, +1 = exactly the UP measurement, 0 = half way.)
     * Frames in between are a hive that is mid-tip, or a bad reading, and are ignored.
     */
    public static final double TIP_MIN_FRAME_SCORE = 0.35;

    /** TUNE: a reading this far outside both the UP and DOWN measurements is thrown away as nonsense. */
    public static final double TIP_MAX_FRAME_SCORE = 2.5;

    /**
     * TUNE: how quickly votes build up (0-1) and how sure the system must be before it changes its
     * mind. With these values a tip is reported after about 4 agreeing camera frames in a row.
     */
    public static final double TIP_VOTE_GAIN = 0.35;
    public static final double TIP_DECIDE_LEVEL = 0.6;

    // =============================================================================================
    // 5. BLENDING THE CAMERA WITH THE ODOMETRY
    // =============================================================================================

    /** TUNE: how well the robot is placed at the start of a match, in inches. */
    public static final double START_POSITION_SIGMA_IN = 2.0;

    /**
     * TUNE: how fast trust in the odometry fades. Bigger numbers make the camera pull the position
     * around more. Raise ODOM_VARIANCE_PER_INCH if the pods slip or the robot gets hit a lot.
     */
    public static final double ODOM_VARIANCE_PER_INCH = 0.002;
    public static final double ODOM_VARIANCE_PER_SECOND = 0.0005;
    public static final double ODOM_MAX_SIGMA_IN = 12.0;

    /**
     * TUNE: how noisy the camera position is, in inches: BASE + PER_RANGE_SQUARED x distance^2.
     * With these values that is about 1.3 in at 3 ft and 2.8 in at 6 ft. Raise both for a worse
     * camera. Watch "camera - fused" in the Sensing Test OpMode while the robot sits still: the
     * scatter you see there is the number to aim for.
     */
    public static final double VISION_SIGMA_BASE_IN = 0.75;
    public static final double VISION_SIGMA_PER_RANGE_SQUARED = 0.0004;

    /** TUNE: the camera is trusted half as much when driving at this speed, or turning at this rate. */
    public static final double VISION_SPEED_FOR_HALF_TRUST_IN_PER_S = 30.0;
    public static final double VISION_TURN_FOR_HALF_TRUST_DEG_PER_S = 60.0;

    /** TUNE: the camera is ignored completely when the robot plus turret turn faster than this, or the hive is further than this. */
    public static final double VISION_MAX_TURN_DEG_PER_S = 200.0;
    public static final double VISION_MAX_RANGE_IN = 110.0;

    /** TUNE: how many of a cluster's 4 tags must be found before it is used (50 = two tags). */
    public static final int VISION_MIN_CLUSTER_PERCENT = 50;

    /**
     * TUNE: a camera reading further than this many "sigmas" from where the robot thinks it is gets
     * rejected as a glitch. 9.21 keeps 99% of honest readings.
     */
    public static final double VISION_GATE_CHI_SQUARED = 9.21;

    /**
     * TUNE: if the camera keeps being rejected but keeps giving the SAME answer (within
     * RELOCK_AGREE_IN) for this many frames in a row, the odometry is the one that is wrong
     * (the robot was shoved or a pod lifted) and the position jumps to the camera's answer.
     */
    public static final int RELOCK_FRAMES = 10;
    public static final double RELOCK_AGREE_IN = 4.0;

    /**
     * TUNE: camera frames older than MAX_FRAME_AGE_MS are not trusted. If the camera's timestamp
     * looks wrong, the frame is assumed to be FALLBACK_LATENCY_MS old instead.
     */
    public static final double MAX_FRAME_AGE_MS = 350.0;
    public static final double FALLBACK_LATENCY_MS = 70.0;

    // =============================================================================================
    // 6. CALIBRATION ("Hive Tag Calibration" OpMode)
    // =============================================================================================

    /**
     * INPUT NEEDED: where the robot is placed for calibration, in field coordinates, with the turret
     * pointing straight ahead. Put it somewhere easy to measure exactly, such as square against a
     * wall on a tile seam, facing the hive.
     */
    public static final double CALIBRATION_ROBOT_X_IN = 72.0;
    public static final double CALIBRATION_ROBOT_Y_IN = 24.0;
    public static final double CALIBRATION_ROBOT_HEADING_DEG = 90.0;
}
