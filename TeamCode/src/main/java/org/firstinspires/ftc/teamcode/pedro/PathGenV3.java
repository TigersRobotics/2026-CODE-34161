package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.HeadingPolicy;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverBuilder;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

public class PathGenV3 {
    // because I don't like the first two. Also, we are likely not going to get a Limelight camera,
    // so things are different.

    private static GoBildaOdometry odometry;
    private static Follower follower;
    private static WeaverBuilder notPathYet;
    private static boolean on;
    private static boolean autonMode;
    private static double timeUntilUpdate;

    public static double tileToIN(double tiles) {
        return tiles * 24;
    }

    public static double INToTile(double inches) {
        return inches / 24;
    }

    private static State currentState = State.IDLE;

    public enum State {
        MOVING_TO_SHOOT,
        SHOOTING,
        PARKING,
        //        COLLECTING,
        IDLE,
        FLOWER_COLLECTING1,
        FLOWER_COLLECTING2
    }

    public static Pose destination;

    public static void init(GoBildaOdometry odometry, Follower follower) {
        PathGenV3.follower = follower;
        PathGenV3.odometry = odometry;

        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH_SHORTENED_CM / 2.54)
                .robotWidth(RobotSpecs.LENGTH_X_M / 2.54)
                .robotHeight(RobotSpecs.HEIGHT_M / 2.54).
                clearance(RobotSpecs.CLEARANCE_IN));
        TimedTelemetry.addLine("PathGenV3 initialized!!! YAY!", 1000);
        timeUntilUpdate = 0;
    }

    public static void update() {
        if (timeUntilUpdate >= 1) {
            timeUntilUpdate -= Timer.DTime();
            return;
        }
        timeUntilUpdate = RobotSpecs.MS_PER_PATH_UPDATE;

        if (follower.isBusy()) TimedTelemetry.addLine("Robot is moving!");
        else TimedTelemetry.addLine("Robot is awaiting commands! :) ");

        if (!on) {
            TimedTelemetry.addLine("Path generation is sleeping... zzz...");
        }
        checkSomeStates();
        if (currentState == State.IDLE) return;
        updateMovement();

    }

    public static void update(boolean pause) {
        if (!pause) {
            update();
        } else {
            follower.stop();
            TimedTelemetry.addLine("Path Gen paused -_-");
        }
    }

    public static void checkSomeStates() {
        switch (currentState){
            case MOVING_TO_SHOOT:
                if (follower.holding()){
                    currentState = State.SHOOTING;
                }
                break;
            case PARKING:
                if (follower.holding()){
                    currentState = State.IDLE;
                }
                break;
            case FLOWER_COLLECTING1:
                if (follower.holding())
                    currentState = State.FLOWER_COLLECTING2;
                break;
        }
    }

    public static void updateMovement() {
        notPathYet = Weaver.builder()
                .start(PedroPathingConverter.toPoint(getCurrentPose()));
        if (destination != null && currentState != State.MOVING_TO_SHOOT)
            notPathYet.end(PedroPathingConverter.toPoint(destination));
        else if (destination != null)
            notPathYet.end(PedroPathingConverter.toPoint(destination), HeadingPolicy.LOCKED);

        for (Zone obstacle : MapSpecs.OBSTACLES) notPathYet.addObstacle(obstacle);
        if (autonMode) notPathYet.addObstacle(MapSpecs.OTHER_SIDE);

        PathResult almostPath = notPathYet.build();
        try {
            follower.follow(PedroPathingConverter.toPath(almostPath.getPath()));
        } catch (NullPointerException | IllegalArgumentException e) {
            TimedTelemetry.addLine("Path Gen failed... (╥﹏╥) (˶˃⤙˂˶)!", 1000);
        }
    }

    public static Pose getCurrentPose() {
        return new Pose(odometry.getX(DistanceUnit.INCH), odometry.getY(DistanceUnit.INCH), odometry.getHeading(AngleUnit.DEGREES));
    }

    public enum Hive {
        LEFT_HIVE,
        RIGHT_HIVE,
        NEAREST
    }

    public static void goShoot(Hive h) {
        TimedTelemetry.addData("goShoot received ^_^ Hive: ", h, 3000);
        currentState = State.MOVING_TO_SHOOT;
        switch (h) {
            case LEFT_HIVE:
                destination = MapSpecs.LEFT_SHOOT_POS;
                break;
            case RIGHT_HIVE:
                destination = MapSpecs.RIGHT_SHOOT_POS;
                break;
            case NEAREST:
                if (getCurrentPose().distance(MapSpecs.LEFT_SHOOT_POS) > getCurrentPose().distance(MapSpecs.RIGHT_SHOOT_POS)) {
                    destination = MapSpecs.RIGHT_SHOOT_POS;
                } else destination = MapSpecs.LEFT_SHOOT_POS;
        }
        TimedTelemetry.addData("New destination: (￣y▽,￣)╭ ", destination, 3000);
    }

    public static void goScoop() {

    }

    //    public static void goCollect(){
//        currentState = State.COLLECTING;
//    }
    public static void goPark() {
        destination = MapSpecs.PARKING;
        if (currentState != State.IDLE && currentState != State.PARKING) {
            TimedTelemetry.addData("Robot has stopped current state to Park (⸝⸝๑  ̫ ๑⸝⸝⸝). Current state: ", currentState, 1000);
        } else if (currentState == State.PARKING)
            TimedTelemetry.addLine("I'm already going to park! ᕙ( ᗒᗣᗕ )ᕗ", 1000);
        currentState = State.PARKING;
    }
}
