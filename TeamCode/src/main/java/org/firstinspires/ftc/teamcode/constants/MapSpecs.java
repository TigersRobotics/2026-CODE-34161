package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.pedro.PathGenV3.tileToIN;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.CircleZone;
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

    public static final PolygonZone LEFT_SHOOTING = new PolygonZone(
            new Point(tileToIN(1.5), tileToIN(2.5 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(0.5), tileToIN(2.5 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(0.5), tileToIN(0.5 - Competition.STARTING_Y_TILES)));
    public static final PolygonZone RIGHT_SHOOTING = new PolygonZone(
            new Point(tileToIN(4.5), tileToIN(2.5 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(5.5), tileToIN(2.5 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(5.5), tileToIN(0.5 - Competition.STARTING_Y_TILES)));

    public static final Pose LEFT_SHOOT_POS = new Pose(tileToIN(1), tileToIN(2.5 - Competition.STARTING_Y_TILES), Math.toRadians(-90));
    public static final Pose RIGHT_SHOOT_POS = new Pose(tileToIN(5), tileToIN(2.5 - Competition.STARTING_Y_TILES), Math.toRadians(90));

    public static final ArrayList<Zone> SHOOTING_ZONES = new ArrayList<>(
            List.of(LEFT_SHOOTING, RIGHT_SHOOTING));

    public static final Pose PARKING = new Pose(tileToIN(1.5), tileToIN(0.5 - Competition.STARTING_Y_TILES), Math.toRadians(90));
    public static final Pose GARDEN = new Pose(tileToIN(5.5), tileToIN(0.5 - Competition.STARTING_Y_TILES));
    public static final ArrayList<Zone> FLOWERS = new ArrayList<>(List.of(
            new CircleZone(new Point(2.5, 48), 2.5),
            new CircleZone(new Point(96, 2.5), 2.5),
            new CircleZone(new Point(141.5, 96), 2.5),
            new CircleZone(new Point(48, 141.5), 2.5)

    ));

    public static final ArrayList<Point> FLOWER_COLLECTION_START = new ArrayList<>(List.of(
            new Point(11, 24 - tileToIN(Competition.STARTING_Y_TILES), Math.toRadians(90)),
            new Point(72, 11 - tileToIN(Competition.STARTING_Y_TILES), Math.toRadians(180))
    ));
    public static final ArrayList<Zone> OBSTACLES = new ArrayList<>(List.of(
            // these are just setting what bounds of the arena is
            new PolygonZone(new Point(tileToIN(0), tileToIN(0 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(6), tileToIN(0 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(3), tileToIN(-2 - Competition.STARTING_Y_TILES))),
            new PolygonZone(new Point(tileToIN(0), tileToIN(0 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(0), tileToIN(6 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(-2), tileToIN(3 - Competition.STARTING_Y_TILES))),
            new PolygonZone(new Point(tileToIN(6), tileToIN(0 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(6), tileToIN(6 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(8), tileToIN(3 - Competition.STARTING_Y_TILES))),
            new PolygonZone(new Point(tileToIN(0), tileToIN(6 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(6), tileToIN(6 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(3), tileToIN(8 - Competition.STARTING_Y_TILES))),

            // the stand of the hives
            new PolygonZone(
                    new Point(46, 47 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(46, 49 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(98, 47 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(98, 49 - tileToIN(Competition.STARTING_Y_TILES))

            ),
            new PolygonZone(
                    new Point(46, 95 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(46, 97 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(98, 95 - tileToIN(Competition.STARTING_Y_TILES)),
                    new Point(98, 97 - tileToIN(Competition.STARTING_Y_TILES))
            )

    )
    );

    public static final Zone OTHER_SIDE = new PolygonZone(
            new Point(tileToIN(3), tileToIN(0 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(6), tileToIN(0 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(6), tileToIN(3 - Competition.STARTING_Y_TILES)),
            new Point(tileToIN(6), tileToIN(6 - Competition.STARTING_Y_TILES))
    );

    public static final Point[] HIVE_COORDINATES =
            {
                    new Point(tileToIN(2.4), tileToIN(2.5 - Competition.STARTING_Y_TILES)),
                    new Point(tileToIN(3.8), tileToIN(2.5 - Competition.STARTING_Y_TILES))
            };


    public static final double HIVE_HEIGHT = 53.5; ///Inches
}
