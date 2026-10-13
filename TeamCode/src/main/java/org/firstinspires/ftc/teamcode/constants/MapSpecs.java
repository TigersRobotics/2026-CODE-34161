package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.pedro.PP.tileToCM;
import static org.firstinspires.ftc.teamcode.pedro.PP.tileToM;

import com.pedropathing.math.Pose;

public class MapSpecs {

    public static final double APRIL_TAG_HIVE_OFFSET_H = 1;
    public static final double APRIL_TAG_HIVE_OFFSET_D = 1;

    public static final Pose LEFT_UPPER_SHOOTING = new Pose(tileToCM(0.5), tileToCM(2.5), -90);
    public static final Pose LEFT_LOWER_SHOOTING = new Pose(tileToCM(0.5), tileToCM(1.5), -90);

    public static final Pose RIGHT_UPPER_SHOOTING = new Pose(tileToCM(5.5), tileToCM(2.5), 90);
    public static final Pose RIGHT_LOWER_SHOOTING = new Pose(tileToM(5.5), tileToCM(1.5), 90);

    public static final Pose PARKING = new Pose(tileToCM(1.5), tileToCM(0.5), 0);
    public static final Pose GARDEN = new Pose(tileToCM(5.5), tileToCM(0.5));


}
