package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.follower.Follower;

public class PedroPathing {

    private static Follower follower;
    public static void init(){
    }

    public static double tileToM(double tiles){
        return tiles*60.95;
    }

    public static double mToTile(double meters){
        return meters/60.95;
    }

}
