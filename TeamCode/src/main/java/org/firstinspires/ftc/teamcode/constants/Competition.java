package org.firstinspires.ftc.teamcode.constants;

import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

public class Competition {
    public static final double STARTING_X_TILES = 0;
    public static final double STARTING_Y_TILES = 0;
    public static final double STARTING_HEADING_DRIVER_OFFSET = 90;

    public static final PolygonZone DISABLED_SIDE = new PolygonZone(
            new Point(3, 0), new Point(3, 3),
            new Point(6, 3), new Point(6, 0) // Change this to new Point(0, 0), new Point(0, 3)
            // if left side needs to be disabled instead of right side
    );
/*
    public static final PolygonZone disabledSide = new PolygonZone(
            new Point(-1,-1), new Point(-2,-2), new Point(-3,-3)
    );
    use this if everywhere is free
*/

}
