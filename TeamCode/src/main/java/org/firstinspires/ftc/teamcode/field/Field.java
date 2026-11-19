package org.firstinspires.ftc.teamcode.field;

/**
 * Where everything is on the BIOBUZZ field.
 *
 * UNITS: inches. Headings in degrees.
 *
 * COORDINATES (looking down at the field, red alliance wall on the left):
 *
 *      (-72, 72) +-----------------------+ (72, 72)  <- blue garden corner
 *                |                       |
 *                |          +y           |
 *      red       |           ^           |       blue
 *      alliance  |           |           |       alliance
 *      wall      |   (0, 0) -+---> +x    |       wall
 *                |                       |
 *                |                       |
 *  red garden -> (-72, -72) -------------+ (72, -72)
 *                     audience side
 *
 * - The field is 144 x 144 in. The origin (0, 0) is the exact centre, under the hive.
 * - +x points at the blue alliance wall, -x at the red alliance wall.
 * - +y points away from the audience, -y toward it.
 * - Every corner is at (+/-72, +/-72).
 * - Tiles are 24 in, so tile seams sit at x or y = -48, -24, 0, 24, 48.
 *
 * HEADING: 0 deg = robot facing +x. Positive is counter-clockwise, so 90 deg = facing +y.
 *
 * ALLIANCES: the field is the same for both alliances, rotated 180 deg.
 * Every position here is written for RED. For blue, (x, y) -> (-x, -y) and heading + 180.
 * Use forAlliance() instead of writing blue numbers by hand.
 *
 * RECTANGLES are stored as {x1, y1, x2, y2}: two opposite corners.
 * POINTS are stored as {x, y}. POSES are stored as {x, y, heading}.
 */
public final class Field {
    public static final double HALF = 72; // in, centre to any wall
    public static final double TILE = 24; // in

    // Robot size, used to work out where the robot's centre is when it is against something.
    public static final double ROBOT_WIDTH = 13; // in, side to side
    public static final double ROBOT_LENGTH = 18; // in, front to back

    // GARDEN: a 23 in strip along the audience wall, starting in red's corner.
    public static final double[] RED_GARDEN = {-72, -72, -49, -70};

    // LOADING ZONE: 23 x 11 in, against red's alliance wall. Parked = any part of the robot inside, not touching the wall.
    public static final double[] RED_LOADING = {-72, 24, -61, 47};

    // FLOWERS: on the walls, 48 in from their corner. All four are listed (no alliance owns one).
    public static final double[][] FLOWERS = {{-72, -24}, {-24, 72}, {72, 24}, {24, -72}};

    // HIVE: one frame in the middle holding two see-saws. Each see-saw tips along y.
    //   Red see-saw pivot is at (-HIVE_X, 0), blue at (+HIVE_X, 0).
    //   The cell that is UP sits CELL_Y from its pivot. Each tip moves the up cell to the other end.
    //   Match start: red's up cell is on the audience side (y < 0), blue's is on the far side (y > 0).
    public static final double HIVE_X = 12; // in
    public static final double CELL_Y = 19; // in
    public static final double PIVOT_HEIGHT = 43.95; // in, above the tiles (manual 9.6)

    // START: where the robot's centre is when the match starts. Fill this in.
    public static final double[] RED_START = {Double.NaN, Double.NaN, 90};

    // PARK: robot's back to the alliance wall with a gap so it does not touch, centred on the loading zone.
    public static final double PARK_GAP = 3; // in, robot edge to wall
    public static final double[] RED_PARK = {-HALF + PARK_GAP + ROBOT_LENGTH / 2, 35.5, 0};

    /** Turns a red position into the same position for the given alliance. Works on points, poses and rectangles. */
    public static double[] forAlliance(double[] red, boolean isRed) {
        if (isRed) return red.clone();
        double[] blue = red.clone();
        if (red.length == 4) {
            blue[0] = -red[0];
            blue[1] = -red[1];
            blue[2] = -red[2];
            blue[3] = -red[3];
        } else {
            blue[0] = -red[0];
            blue[1] = -red[1];
            if (red.length == 3) blue[2] = red[2] + 180;
        }
        return blue;
    }

    /** The cell to shoot at: the one that is up. tipped = false at match start, flip it each time the hive tips. */
    public static double[] upCell(boolean isRed, boolean tipped) {
        double x = isRed ? -HIVE_X : HIVE_X;
        double y = isRed ? -CELL_Y : CELL_Y;
        return new double[] {x, tipped ? -y : y};
    }

    public static double[] start(boolean isRed) {
        return forAlliance(RED_START, isRed);
    }

    public static double[] park(boolean isRed) {
        return forAlliance(RED_PARK, isRed);
    }

    public static double[] loadingZone(boolean isRed) {
        return forAlliance(RED_LOADING, isRed);
    }

    public static double[] garden(boolean isRed) {
        return forAlliance(RED_GARDEN, isRed);
    }

    public static double toMeters(double inches) {
        return inches * 0.0254;
    }
}
