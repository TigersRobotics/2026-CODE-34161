package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
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

public class PathGenV2 {
    /// because I don't like the original one

    private static Telemetry telemetry;
    private static Odometry odometry;
    private static Follower follower;
    private static WeaverBuilder notPathYet;
    private static boolean on = true;

    public static double tileToIN(double tiles){
        return tiles*24;
    }

    public static double INToTile(double inches){
        return inches/24;
    }

    private enum State {
        SHOOTING,
        PARKING,
        COLLECTING,
        IDLE,
        HIJACKED
    }

    private static State currentState = State.IDLE;


    private static boolean autoMode = true;
    private static Pose destination = null;
    private static ArrayList<Zone> obstacles = new ArrayList<>();
    private static ArrayList<Point> targets = new ArrayList<>();

    public static boolean isAutoMode() {
        return autoMode;
    }

    public static void setAutoMode(boolean autoMode) {
        PathGenV2.autoMode = autoMode;
    }

    public static Pose getDestination() {
        return destination;
    }

    public static void setDestination(Pose destination) {
        PathGenV2.destination = destination;
    }

    public static ArrayList<Zone> getObstacles() {
        return obstacles;
    }

    public static void setObstacles(ArrayList<Zone> obstacles) {
        PathGenV2.obstacles = obstacles;
    }

    public static ArrayList<Point> getTargets() {
        return targets;
    }

    public static void setTargets(ArrayList<Point> targets) {
        PathGenV2.targets = targets;
    }

    public static Pose getCurrentPose(){
        return new Pose(odometry.getX(DistanceUnit.INCH), odometry.getY(DistanceUnit.INCH), odometry.getHeading(AngleUnit.DEGREES));
    }

    public static void init(Odometry odometry, Telemetry telemetry, Follower follower){
        PathGenV2.follower = follower;
        PathGenV2.odometry = odometry;
        PathGenV2.telemetry = telemetry;
        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH_SHORTENED /2.54)
                .robotWidth(RobotSpecs.LENGTH_X/2.54)
                .robotHeight(RobotSpecs.HEIGHT/2.54).clearance(RobotSpecs.CLEARANCE));
        telemetry.addLine("PathGenV2 initialized!!! YAY!");
    }

    public static void update(ArrayList<Zone> obstacles){
        if (destination == null){
            if (currentState == State.IDLE){
                telemetry.addLine("Path gen is idling! ٩(^ᗜ^ )");
            } else if (currentState == State.HIJACKED) {
                telemetry.addLine("Path Gen is HIJACKED! No auto mode for you! Bleh!");
            } else telemetry.addData("Weird... no destination, but not idle. Ehh!? " +
                    "Current state is:", currentState);
            if (follower.isBusy()){
                follower.stop();
            }
        }


        if (follower.isBusy()) telemetry.addLine("Robot-chan is moving!");
         else telemetry.addLine("Robot-chan is awaiting commands! :) ");

        if (!on) {
            telemetry.addLine("Path generation is sleeping... zzz...");
        }

        PathGenV2.notPathYet = Weaver.builder()
                .start(PedroPathingConverter.toPoint(getCurrentPose()));
        if (destination!= null) notPathYet.end(PedroPathingConverter.toPoint(destination));

        for (Zone obstacle: MapSpecs.OBSTACLES) notPathYet.addObstacle(obstacle);
        for (Zone obstacle: obstacles) notPathYet.addObstacle(obstacle);
        for (Zone obstacle: PathGenV2.obstacles) notPathYet.addObstacle(obstacle);
        for (Point target: targets) notPathYet.addTarget(target);
        if (autoMode) notPathYet.addObstacle(MapSpecs.OTHER_SIDE);


        PathResult almostPath = notPathYet.build();
        try {
            follower.follow(PedroPathingConverter.toPath(almostPath.getPath()));
        } catch (NullPointerException | IllegalArgumentException e){
            telemetry.addLine("Path Gen failed... (╥﹏╥)");
        }

        if (currentState == State.SHOOTING){
            for (Zone shootingZone: MapSpecs.SHOOTING_POSITIONS){

            }
        }
    }

    public static boolean setOn(boolean on){
        /// sets the path follower on or off
        /// returns true if the path is switched,
        /// otherwise return false
        if (on==PathGenV2.on){
            if (on) telemetry.addLine("Path Gen is already wide awake! :3");
            else telemetry.addLine("Path Gen is already sleeping. Hmph!");
            return false;
        } else {
            if (on){
                PathGenV2.on = true;
                telemetry.addLine("Awaking up!");
            } else {
                PathGenV2.on = false;
                telemetry.addLine("Sleeeepping...");
            }
            return true;
        }
    }
    public static boolean getBusy(){
        return follower.isBusy();
    }

    public static void goShoot(){
        Pose nearestSpot = getCurrentPose();
        double closest = 1000;
        for (Zone shootingSpot: MapSpecs.SHOOTING_POSITIONS){
            if (shootingSpot.distanceTo(PedroPathingConverter.toPoint(getCurrentPose())) < closest){
                closest = shootingSpot.distanceTo(PedroPathingConverter.toPoint(getCurrentPose()));
                nearestSpot = new Pose(shootingSpot.getPosition().getX(), shootingSpot.getPosition().getY());
            }
        }

        setDestination(nearestSpot);
        currentState = State.SHOOTING;
        telemetry.addData("Going to shoot! ₍₍⚞(˶˃ ᵕ ˂˶)⚟⁾⁾. Going to:", nearestSpot);
    }



}
