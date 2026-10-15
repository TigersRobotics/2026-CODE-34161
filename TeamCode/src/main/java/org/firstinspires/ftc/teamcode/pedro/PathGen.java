package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverBuilder;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.sensors.Odometry;

import java.util.ArrayList;

public class PathGen {
    /// the pedropathing file

    private static Follower follower;
    private static Odometry odometry;
    private static Telemetry telemetry;
    public static void init(Odometry odometry, Telemetry telemetry){
        PathGen.odometry = odometry;
        PathGen.telemetry = telemetry;
        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH/2.54)
                .robotWidth(RobotSpecs.LENGTH_X/2.54)
                .robotHeight(RobotSpecs.HEIGHT/2.54));
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


    public static boolean goTo(Pose pose, ArrayList<Point> targets){
        if (follower.isBusy()){
            telemetry.addLine("Robot is busy, goTo() method stopped.");
            return false;
        }

        WeaverBuilder notPath = Weaver.builder()
                .start(PedroPathingConverter.toPoint(getCurrentPose()))
                .end(PedroPathingConverter.toPoint(pose));
        for (Zone obstacle: MapSpecs.obstacles) notPath.addObstacle(obstacle);
        PathResult builtPath = notPath.build();

        try {
            Path path = PedroPathingConverter.toPath(builtPath);

            follower.follow(path);
        } catch (NullPointerException | IllegalArgumentException e){
            telemetry.addLine("No path found, goTo() method stopped.");
            return false;
        }
        return true;
    }

    public static Pose getCurrentPose (){
        return new Pose(odometry.getX(DistanceUnit.CM), odometry.getY(DistanceUnit.CM), odometry.getHeading(AngleUnit.DEGREES));
    }

}
