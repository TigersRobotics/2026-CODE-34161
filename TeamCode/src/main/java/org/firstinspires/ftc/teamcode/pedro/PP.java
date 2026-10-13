package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.sensors.Odometry;

public class PP {
    /// the pedropathing file

    private static Follower follower;
    private static Odometry odometry;
    public static void init(Odometry odometry){
        PP.odometry = odometry;
    }

    public static double tileToM(double tiles){
        return tiles*60.95;
    }

    public static double mToTile(double meters){
        return meters/60.95;
    }

    public static double tileToCM(double tiles){
        return tiles*6095;
    }

    public static double cmToTile(double centimeters){
        return centimeters/6095;
    }

    public static boolean goTo(Pose pose){
        if (follower.isBusy()) return false;

    }

}
