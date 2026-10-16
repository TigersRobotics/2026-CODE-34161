package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.pedro.PathGen.tileToIN;


import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;


import java.util.ArrayList;
import java.util.List;

public class MapSpecs {
/// this is a class containing constants of the map, such as the hive entrance/april tag offset,
/// and the locations of various POIs
    public static final double APRIL_TAG_HIVE_OFFSET_H = 1;
    public static final double APRIL_TAG_HIVE_OFFSET_D = 1;

    public static final Pose LEFT_UPPER_SHOOTING = new Pose(tileToIN(0.5), tileToIN(2.5), -90);
    public static final Pose LEFT_LOWER_SHOOTING = new Pose(tileToIN(0.5), tileToIN(1.5), -90);

    public static final Pose RIGHT_UPPER_SHOOTING = new Pose(tileToIN(5.5), tileToIN(2.5), 90);
    public static final Pose RIGHT_LOWER_SHOOTING = new Pose(tileToIN(5.5), tileToIN(1.5), 90);

    public static final ArrayList<Pose> SHOOTING_POSITIONS = new ArrayList<>(
            List.of(LEFT_UPPER_SHOOTING, LEFT_LOWER_SHOOTING, RIGHT_UPPER_SHOOTING, RIGHT_LOWER_SHOOTING));
    public static final Pose PARKING = new Pose(tileToIN(1.5), tileToIN(0.5), 0);
    public static final Pose GARDEN = new Pose(tileToIN(5.5), tileToIN(0.5));

    public static final ArrayList<Zone> OBSTACLES = new ArrayList<>(List.of(
                    // enter the obstacles here
            ));

    public static final Zone OTHER_SIDE = new PolygonZone(
            new Point(tileToIN(3), tileToIN(0)), new Point(tileToIN(6), tileToIN(0)),
            new Point(tileToIN(6), tileToIN(3)), new Point(tileToIN(6), tileToIN(6))
    );
}
