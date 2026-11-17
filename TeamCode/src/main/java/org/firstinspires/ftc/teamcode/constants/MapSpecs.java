package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.pedro.PathGenV2.tileToIN;


import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class MapSpecs {
    /// this is a class containing constants of the map, such as the hive entrance/april tag offset,
    /// and the locations of various POIs
    public static final double APRIL_TAG_HIVE_OFFSET_H = 1;
    public static final double APRIL_TAG_HIVE_OFFSET_D = 1;

    public static final PolygonZone LEFT_SHOOTING = new PolygonZone(
            new Point(tileToIN(1.5), tileToIN(2.5)),
            new Point(tileToIN(0.5), tileToIN(2.5)),
            new Point(tileToIN(0.5), tileToIN(0.5)));
    public static final PolygonZone RIGHT_SHOOTING = new PolygonZone(
            new Point(tileToIN(4.5), tileToIN(2.5)),
            new Point(tileToIN(5.5), tileToIN(2.5)),
            new Point(tileToIN(5.5), tileToIN(0.5)));

    public static final Pose LEFT_SHOOT_POS = new Pose(tileToIN(1), tileToIN(2.5), -90);
    public static final Pose RIGHT_SHOOT_POS = new Pose(tileToIN(5), tileToIN(2.5), 90);

    public static final ArrayList<Zone> SHOOTING_ZONES = new ArrayList<>(
            List.of(LEFT_SHOOTING, RIGHT_SHOOTING));

    public static final Pose PARKING = new Pose(tileToIN(1.5), tileToIN(0.5), 90);
    public static final Pose GARDEN = new Pose(tileToIN(5.5), tileToIN(0.5));

    public static final ArrayList<Zone> OBSTACLES = new ArrayList<>(List.of(
            new PolygonZone(new Point(0, 0), new Point(6, 0), new Point(3, -2)),
            new PolygonZone(new Point(0, 0), new Point(0,6), new Point(-2, 3)),
            new PolygonZone(new Point(6,0), new Point(6,6), new Point(8, 3)),
            new PolygonZone(new Point(0,6), new Point(6,6), new Point(3,8))
    ));

    public static final Zone OTHER_SIDE = new PolygonZone(
            new Point(tileToIN(3), tileToIN(0)), new Point(tileToIN(6), tileToIN(0)),
            new Point(tileToIN(6), tileToIN(3)), new Point(tileToIN(6), tileToIN(6))
    );

    public static final Point[] HIVE_COORDINATES =
            {
                    new Point(tileToIN(2.4), tileToIN(2.5)),
                    new Point(tileToIN(3.8), tileToIN(2.5))
            };


    public static final double HIVE_HEIGHT = 53.5; ///Inches
}
