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
import java.util.List;

public class PathGen {
    /// the pedropathing file

    private static Follower follower;
    private static Odometry odometry;
    private static Telemetry telemetry;
    private static boolean autoMode = true;

    public static Pose getDestination() {
        return destination;
    }

    public static void setDestination(Pose destination) {
        PathGen.destination = destination;
    }

    public static ArrayList<Point> getTargets() {
        return targets;
    }

    public static void setTargets(ArrayList<Point> targets) {
        PathGen.targets = targets;
    }

    public static ArrayList<Zone> getObstacles() {
        return obstacles;
    }

    public static void setObstacles(ArrayList<Zone> obstacles) {
        PathGen.obstacles = obstacles;
    }

    private static ArrayList<Zone> obstacles = new ArrayList<>();
    private static ArrayList<Point> targets = new ArrayList<>();
    private static Pose destination;
    public static void init(Odometry odometry, Telemetry telemetry){
        PathGen.odometry = odometry;
        PathGen.telemetry = telemetry;
        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH/2.54)
                .robotWidth(RobotSpecs.LENGTH_X/2.54)
                .robotHeight(RobotSpecs.HEIGHT/2.54));
    }

    public static void update(ArrayList<Zone> obstacles){
        PathGen.obstacles = obstacles;
        follower.update();
    }

    public static void setAutoMode(boolean autoMode){
        PathGen.autoMode = autoMode;
    }



    public static double tileToIN(double tiles){
        return tiles*24;
    }

    public static double INToTile(double inches){
        return inches/24;
    }


    public static boolean goTo(Pose destination, ArrayList<Zone> obstacles, ArrayList<Point> targets){
        if (follower.isBusy()){
            telemetry.addLine("Robot is busy, goTo() method stopped.");
            return false;
        }

        WeaverBuilder notPath = Weaver.builder()
                .start(PedroPathingConverter.toPoint(getCurrentPose()))
                .end(PedroPathingConverter.toPoint(destination));

        for (Zone obstacle: MapSpecs.OBSTACLES) notPath.addObstacle(obstacle);
        for (Zone obstacle: obstacles) notPath.addObstacle(obstacle);
        for (Zone obstacle: PathGen.obstacles) notPath.addObstacle(obstacle);

        if (autoMode) notPath.addObstacle(MapSpecs.OTHER_SIDE);

        PathResult builtPath = notPath.build();
        for (Point target: targets) notPath.addTarget(target);
        try {
            Path path = PedroPathingConverter.toPath(builtPath);
            follower.follow(path);
        } catch (NullPointerException | IllegalArgumentException e){
            telemetry.addLine("No path found, goTo() method stopped.");
            return false;
        }
        return true;
    }

    public static boolean goTo(Pose destination){
        return goTo(destination, obstacles, new ArrayList<>());
    }

    public static boolean goTo(Pose destination, ArrayList<Zone> avoid){
        return goTo(destination, avoid, new ArrayList<>(List.of()));
    }


    public static void stopFollowPath(){
        follower.stop();
    }

    public static Pose getCurrentPose (){
        return new Pose(odometry.getX(DistanceUnit.CM), odometry.getY(DistanceUnit.CM), odometry.getHeading(AngleUnit.DEGREES));
    }

    public static void goShoot(){
        if (follower.isBusy()) {
            telemetry.addLine("Robot busy, unable to execute goShoot()");
            return;
        }

        Pose nearestSpot = getCurrentPose();
        double closest = 1000;
        for (Pose spot: MapSpecs.SHOOTING_POSITIONS){
            if (spot.distance(getCurrentPose()) < closest){
                closest = spot.distance(getCurrentPose());
                nearestSpot = spot;
            }
        }
        if(goTo(nearestSpot)) telemetry.addLine("Following path to shooting");
        else telemetry.addLine("Path finding failed. goShoot() -> goTo(nearestSpot)");
    }
}
