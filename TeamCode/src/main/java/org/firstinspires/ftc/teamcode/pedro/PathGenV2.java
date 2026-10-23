package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.HeadingPolicy;
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
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

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
    public enum State {
        MOVING_TO_SHOOT,
        SHOOTING,
        PARKING,
        COLLECTING,
        IDLE,
    }

    private static State currentState = State.IDLE;


    private static boolean autoMode = true;
    private static Pose destination = null;
    private static ArrayList<Zone> obstacles = new ArrayList<>();
    private static ArrayList<Point> scoreElements = new ArrayList<>();

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

    public static ArrayList<Point> getCurrentTrackedElements() {
        return scoreElements;
    }

    public static void collectElements(ArrayList<Point> scoreElements) {
        currentState=State.COLLECTING;
        PathGenV2.scoreElements = scoreElements;
    }

    public static Pose getCurrentPose(){
        return new Pose(odometry.getX(DistanceUnit.INCH), odometry.getY(DistanceUnit.INCH), odometry.getHeading(AngleUnit.DEGREES));
    }

    public static void init(Odometry odometry, Telemetry telemetry, Follower follower, boolean debug){
        PathGenV2.follower = follower;
        PathGenV2.odometry = odometry;
        PathGenV2.telemetry = telemetry;
        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH_SHORTENED /2.54)
                .robotWidth(RobotSpecs.LENGTH_X/2.54)
                .robotHeight(RobotSpecs.HEIGHT/2.54).clearance(RobotSpecs.CLEARANCE));
        TimedTelemetry.addLine("PathGenV2 initialized!!! YAY!", 1000);
    }

    public static void init(Odometry odometry, Telemetry telemetry, Follower follower){
        init(odometry, telemetry, follower, false);
    }

    public static void update(ArrayList<Zone> obstacles){


        if (follower.isBusy()) TimedTelemetry.addLine("Robot-chan is moving!");
         else TimedTelemetry.addLine("Robot-chan is awaiting commands! :) ");

        if (!on) {
            TimedTelemetry.addLine("Path generation is sleeping... zzz...");
        }
        updateMovement(obstacles);
    }
    public static void update(ArrayList<Zone> obstacles, boolean pause){
        if (!pause){
            update(obstacles);
        } else {
            follower.stop();
            TimedTelemetry.addLine("Path Gen paused");
        }
    }

    private static void updateMovement(ArrayList<Zone> obstacles){



        PathGenV2.notPathYet = Weaver.builder()
                .start(PedroPathingConverter.toPoint(getCurrentPose()));
        if (destination!= null && currentState!=State.MOVING_TO_SHOOT) notPathYet.end(PedroPathingConverter.toPoint(destination));
        else if (destination != null) notPathYet.end(PedroPathingConverter.toPoint(destination), HeadingPolicy.LOCKED);
        for (Zone obstacle: MapSpecs.OBSTACLES) notPathYet.addObstacle(obstacle);
        for (Zone obstacle: obstacles) notPathYet.addObstacle(obstacle);
        for (Zone obstacle: PathGenV2.obstacles) notPathYet.addObstacle(obstacle);
        if (currentState!=State.MOVING_TO_SHOOT) for (Point target: scoreElements) notPathYet.addTarget(target);

        if (autoMode) notPathYet.addObstacle(MapSpecs.OTHER_SIDE);

        PathResult almostPath = notPathYet.build();
        try {
            follower.follow(PedroPathingConverter.toPath(almostPath.getPath()));
        } catch (NullPointerException | IllegalArgumentException e){
            TimedTelemetry.addLine("Path Gen failed... (╥﹏╥) (˶˃⤙˂˶)!", 1000);
        }
    }

    private static void updateSomeStates(){
        switch (currentState){
            case PARKING:
                if (!follower.isBusy()|| destination.distance(getCurrentPose()) < 0.5){
                    destination= null;
                    currentState = State.IDLE;
                }
                break;
            case MOVING_TO_SHOOT:
                if (!follower.isBusy()|| destination.distance(getCurrentPose()) < 0.5){
                    currentState = State.SHOOTING;
                    destination = null;
                }
                break;
            case COLLECTING:
                destination = null;

        }
        TimedTelemetry.addData("Current state: ", currentState);
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
        double targetHeading;
        Pose nearestSpot = getCurrentPose();
        double closest = 1000;
        for (Zone shootingSpot: MapSpecs.SHOOTING_POSITIONS){
            if (shootingSpot.distanceTo(PedroPathingConverter.toPoint(getCurrentPose())) < closest){
                closest = shootingSpot.distanceTo(PedroPathingConverter.toPoint(getCurrentPose()));
                nearestSpot = new Pose(shootingSpot.getPosition().getX(), shootingSpot.getPosition().getY());
            }
        }

        if (INToTile(nearestSpot.x()) < 3){
            targetHeading = -90;
        } else {
            targetHeading = 90;
        }

        setDestination(new Pose(nearestSpot.x(),nearestSpot.y(), targetHeading ));
        currentState = State.MOVING_TO_SHOOT;
        TimedTelemetry.addData("Going to shoot! ₍₍⚞(˶˃ ᵕ ˂˶)⚟⁾⁾. Going to: ", nearestSpot, 1000);
    }

    public static void goPark(){
        destination = MapSpecs.PARKING;
        if (currentState != State.IDLE && currentState != State.PARKING){
            TimedTelemetry.addData("Robot-chan has stopped current state to Park (⸝⸝๑  ̫ ๑⸝⸝⸝). Current state: ", currentState, 1000);
        } else if(currentState == State.PARKING) TimedTelemetry.addLine("I'm already going to park! ᕙ( ᗒᗣᗕ )ᕗ",1000);
        currentState = State.PARKING;
    }

    public static State getCurrentState(){
        return currentState;
    }

}
