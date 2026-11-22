package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;

public class MapSpecs {
    /// this is a class containing constants of the map, such as the hive entrance/april tag offset,
    /// and the locations of various POIs

    public static final Zone[] OBSTACLES = {
            // Walls
            new PolygonZone(
                    new Point(-24, 0 - Competition.STARTING_Y_INCHES),
                    new Point(168, 0 - Competition.STARTING_Y_INCHES),
                    new Point(168, -48 - Competition.STARTING_Y_INCHES),
                    new Point(-24, -48 - Competition.STARTING_Y_INCHES)
            ),
            new PolygonZone(
                    new Point(0, -24 - Competition.STARTING_Y_INCHES),
                    new Point(0, 168 - Competition.STARTING_Y_INCHES),
                    new Point(-48, 168 - Competition.STARTING_Y_INCHES),
                    new Point(-48, -24 - Competition.STARTING_Y_INCHES)
            ),
            new PolygonZone(
                    new Point(144, -24 - Competition.STARTING_Y_INCHES),
                    new Point(144, 168 - Competition.STARTING_Y_INCHES),
                    new Point(192, 168 - Competition.STARTING_Y_INCHES),
                    new Point(192, -24 - Competition.STARTING_Y_INCHES)
            ),
            new PolygonZone(
                    new Point(-24, 144 - Competition.STARTING_Y_INCHES),
                    new Point(168, 144 - Competition.STARTING_Y_INCHES),
                    new Point(168, 192 - Competition.STARTING_Y_INCHES),
                    new Point(-24, 192 - Competition.STARTING_Y_INCHES)
            )
    };

    public static final Zone[] HIVE_BOTTOMS = {
            // Hive Bottoms
            new PolygonZone(
                    new Point(52.8, 47 - Competition.STARTING_Y_INCHES),
                    new Point(90.7, 47 - Competition.STARTING_Y_INCHES),
                    new Point(90.7, 49 - Competition.STARTING_Y_INCHES),
                    new Point(52.8, 49 - Competition.STARTING_Y_INCHES)
            ),
            new PolygonZone(
                    new Point(52.8, 95 - Competition.STARTING_Y_INCHES),
                    new Point(90.7, 95 - Competition.STARTING_Y_INCHES),
                    new Point(90.7, 97 - Competition.STARTING_Y_INCHES),
                    new Point(52.8, 97 - Competition.STARTING_Y_INCHES)
            )
    };

    public static final Zone[] FLOWERS = {
            // Left wall flower
            new PolygonZone(
                    new Point(0, 37.5 - Competition.STARTING_Y_INCHES),
                    new Point(5, 37.5 - Competition.STARTING_Y_INCHES),
                    new Point(5, 42.5 - Competition.STARTING_Y_INCHES),
                    new Point(0, 42.5 - Competition.STARTING_Y_INCHES)
            ),
            // Top wall flower
            new PolygonZone(
                    new Point(45.5, 139 - Competition.STARTING_Y_INCHES),
                    new Point(50.5, 139 - Competition.STARTING_Y_INCHES),
                    new Point(50.5, 144 - Competition.STARTING_Y_INCHES),
                    new Point(45.5, 144 - Competition.STARTING_Y_INCHES)
            ),
            // Right wall flower
            new PolygonZone(
                    new Point(139, 93.5 - Competition.STARTING_Y_INCHES),
                    new Point(144, 93.5 - Competition.STARTING_Y_INCHES),
                    new Point(144, 98.5 - Competition.STARTING_Y_INCHES),
                    new Point(139, 98.5 - Competition.STARTING_Y_INCHES)
            ),
            // Bottom wall flower
            new PolygonZone(
                    new Point(93.5, 0 - Competition.STARTING_Y_INCHES),
                    new Point(98.5, 0 - Competition.STARTING_Y_INCHES),
                    new Point(98.5, 5 - Competition.STARTING_Y_INCHES),
                    new Point(93.5, 5 - Competition.STARTING_Y_INCHES)
            )
    };


    public static final Pose PARKING = new Pose(36, 6);

    public static final Zone PARKING_ZONE =
            // Bottom-left loading zone
            new PolygonZone(
                    new Point(24, 0 - Competition.STARTING_Y_INCHES),
                    new Point(48, 0 - Competition.STARTING_Y_INCHES),
                    new Point(48, 12 - Competition.STARTING_Y_INCHES),
                    new Point(24, 12 - Competition.STARTING_Y_INCHES)
            );
    public static final Zone OPP_PARKING_ZONE =
            // Top-right loading zone
            new PolygonZone(
                    new Point(96, 132 - Competition.STARTING_Y_INCHES),
                    new Point(120, 132 - Competition.STARTING_Y_INCHES),
                    new Point(120, 144 - Competition.STARTING_Y_INCHES),
                    new Point(96, 144 - Competition.STARTING_Y_INCHES)
            );


    public static final Zone[] GARDENS = {
            // Bottom-right garden
            new PolygonZone(
                    new Point(139.2, 0 - Competition.STARTING_Y_INCHES),
                    new Point(144, 0 - Competition.STARTING_Y_INCHES),
                    new Point(144, 24 - Competition.STARTING_Y_INCHES),
                    new Point(139.2, 24 - Competition.STARTING_Y_INCHES)
            ),
            // Top-left garden
            new PolygonZone(
                    new Point(0, 120 - Competition.STARTING_Y_INCHES),
                    new Point(4.8, 120 - Competition.STARTING_Y_INCHES),
                    new Point(4.8, 144 - Competition.STARTING_Y_INCHES),
                    new Point(0, 144 - Competition.STARTING_Y_INCHES)
            )
    };

    public static final Zone[] SHOOTING_ZONES = {
            // Left shooting zone
            new PolygonZone(
                    new Point(0, 72 - Competition.STARTING_Y_INCHES),
                    new Point(40, 72 - Competition.STARTING_Y_INCHES),
                    new Point(40, 40 - Competition.STARTING_Y_INCHES),
                    new Point(0, 0 - Competition.STARTING_Y_INCHES)
            ),
            // Right shooting zone
            new PolygonZone(
                    new Point(104, 72 - Competition.STARTING_Y_INCHES),
                    new Point(144, 72 - Competition.STARTING_Y_INCHES),
                    new Point(144, 0 - Competition.STARTING_Y_INCHES),
                    new Point(104, 40 - Competition.STARTING_Y_INCHES)
            )
    };

    public static final Zone OTHER_SIDE = new PolygonZone(
            new Point(144, 144 - Competition.STARTING_Y_INCHES),
            new Point(0, 144 - Competition.STARTING_Y_INCHES),
            new Point(0, 72 - Competition.STARTING_Y_INCHES),
            new Point(144, 72 - Competition.STARTING_Y_INCHES)
    );

    public static final Point[] FLOWER_COLLECTION_START = {
            // Left wall flower -> facing left (180°)
            new Point(16, 40 - Competition.STARTING_Y_INCHES, Math.toRadians(180)),
            // Top wall flower -> facing up (90°)
            new Point(48, 128 - Competition.STARTING_Y_INCHES, Math.toRadians(90)),
            // Right wall flower -> facing right (0°)
            new Point(128, 96 - Competition.STARTING_Y_INCHES, Math.toRadians(0)),
            // Bottom wall flower -> facing down (-90°)
            new Point(96, 16 - Competition.STARTING_Y_INCHES, Math.toRadians(-90))
    };

    public static final Point[] HIVE_COORDINATES = {
            new Point(48, 60 - Competition.STARTING_Y_INCHES),
            new Point(96, 60 - Competition.STARTING_Y_INCHES)
    };

    public static final Pose LEFT_SHOOT_POS = new Pose(9, 60 - Competition.STARTING_Y_INCHES, Math.toRadians(0));
    public static final Pose RIGHT_SHOOT_POS = new Pose(135, 60 - Competition.STARTING_Y_INCHES, Math.toRadians(180));


    public static final double HIVE_HEIGHT = 53.5; ///Inches
}
