package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverBuilder;
import com.skeletonarmy.marrow.zones.Zone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Competition;
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
    private static boolean on;
    public static void setOn (boolean on){
        PathGenV3.on = on;
    }

    private static boolean autonMode;
    private static double timeUntilUpdate;

    public static double tileToIN(double tiles) {
        return tiles * 24;
    }

    public static double INToTile(double inches) {
        return inches / 24;
    }

    private static State currentState = State.IDLE;
    private static final State previousState = State.IDLE;


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

    public static void init(GoBildaOdometry odometry, Follower follower, boolean autonMode) {
        PathGenV3.autonMode = autonMode;
        on = autonMode;
        currentState = State.IDLE;
        PathGenV3.follower = follower;
        follower.setPose(new Pose(tileToIN(Competition.STARTING_X_TILES), 0, 0));
        PathGenV3.odometry = odometry;

        Weaver.setConfig(new PathConfig().intakeWidth(RobotSpecs.INTAKE_WIDTH_SHORTENED_CM / 2.54)
                .robotWidth(RobotSpecs.LENGTH_X_M / 2.54)
                .robotHeight(RobotSpecs.HEIGHT_M / 2.54).
                clearance(RobotSpecs.CLEARANCE_IN));
        TimedTelemetry.addLine("PathGenV3 initialized!!! YAY!", 1000);
        timeUntilUpdate = 0;
    }

    public static void update() {
        follower.update(Timer.DTime());
        TimedTelemetry.addData("Current State", currentState);
        TimedTelemetry.addData("Previous State", previousState);
        TimedTelemetry.addData("Follower Stats", follower.following());
        TimedTelemetry.addData("Current Pose", getCurrentPose());
        TimedTelemetry.addData("Destination", destination);
        if (timeUntilUpdate >= 1) {
            timeUntilUpdate -= Timer.DTime();
            return;
        }
        timeUntilUpdate = RobotSpecs.MS_PER_PATH_UPDATE;

        if (follower.isBusy())
            TimedTelemetry.addLine("Robot is moving!", (int) (RobotSpecs.MS_PER_PATH_UPDATE - 20));
        else
            TimedTelemetry.addLine("Robot is awaiting commands! :) ", (int) (RobotSpecs.MS_PER_PATH_UPDATE - 20));

        if (!on) {
            TimedTelemetry.addLine("Path generation is sleeping... zzz...");
        }
        checkSomeStates();
        TimedTelemetry.addData("Current State", currentState);
        TimedTelemetry.addData("Previous State", previousState);
        TimedTelemetry.addData("Follower Stats", follower.following());
        if (currentState == State.IDLE) return;
        if (currentState != previousState) {
            currentState = previousState;
            updateMovement();
        }

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
//        switch (currentState) {
//            case MOVING_TO_SHOOT:
//                if (follower.holding()) {
//                    currentState = State.SHOOTING;
//                }
//                break;
//            case PARKING:
//                if (follower.holding()) {
//                    currentState = State.IDLE;
//                }
//                break;
//            case FLOWER_COLLECTING1:
//                if (follower.holding())
//                    currentState = State.FLOWER_COLLECTING2;
//                break;
//        }
    }

    public static void updateMovement() {
        WeaverBuilder notPathYet = Weaver.builder();
        notPathYet.start(PedroPathingConverter.toPoint(getCurrentPose()));
        for (Zone obstacle : MapSpecs.OBSTACLES) notPathYet.addObstacle(obstacle);
        if (autonMode) notPathYet.addObstacle(MapSpecs.OTHER_SIDE).addObstacle(Competition.DISABLED_SIDE);
        notPathYet.end(PedroPathingConverter.toPoint(destination));
        PathResult almostPath = notPathYet.build();
        try {
            TimedTelemetry.addLine("Follower followed", 1000);
            follower.follow(PedroPathingConverter.toPath(almostPath.getPath()));
        } catch (NullPointerException | IllegalArgumentException e) {
            TimedTelemetry.addLine("Path Gen failed... (╥﹏╥) (˶˃⤙˂˶)!", 1000);
        }
    }

    public static Pose getCurrentPose() {
        return new Pose(odometry.getX(DistanceUnit.INCH),
                odometry.getY(DistanceUnit.INCH),
                odometry.getHeading(AngleUnit.DEGREES));
    }

    public enum Hive {
        LEFT_HIVE,
        RIGHT_HIVE,
        NEAREST
    }

    public static void goShoot(Hive h) {
        TimedTelemetry.addData("goShoot received ^_^ Hive: ", h, 3000);
        switch (h) {
            case LEFT_HIVE:
                destination = MapSpecs.LEFT_SHOOT_POS;
                if (autonMode&&Competition.DISABLED_SIDE.contains(PedroPathingConverter.toPoint(destination))) {
                    destination = null;
                }
                break;
            case RIGHT_HIVE:
                destination = MapSpecs.RIGHT_SHOOT_POS;
                if (autonMode&&Competition.DISABLED_SIDE.contains(PedroPathingConverter.toPoint(destination))) {
                    destination = null;
                }
                break;
            case NEAREST:
                if (getCurrentPose().distance(MapSpecs.LEFT_SHOOT_POS) > getCurrentPose().distance(MapSpecs.RIGHT_SHOOT_POS)) {
                    destination = MapSpecs.RIGHT_SHOOT_POS;
                } else destination = MapSpecs.LEFT_SHOOT_POS;
                break;
        }
        if (destination == null){
            TimedTelemetry.addLine("Invalid shooting position given. ", 3000);
            return;
        }
        currentState = State.MOVING_TO_SHOOT;



        TimedTelemetry.addData("New destination: (￣y▽,￣)╭ ", destination, 3000);
    }

    public static void goScoop() {

    }

    //    public static void goCollect(){
//        currentState = State.COLLECTING;
//    } Unused for now

    public static void goPark() {
        destination = MapSpecs.PARKING;
        if (currentState != State.IDLE && currentState != State.PARKING) {
            TimedTelemetry.addData("Robot has stopped current state to Park (⸝⸝๑  ̫ ๑⸝⸝⸝). Current state: ", currentState, 1000);
        } else if (currentState == State.PARKING)
            TimedTelemetry.addLine("I'm already going to park! ᕙ( ᗒᗣᗕ )ᕗ", 1000);
        currentState = State.PARKING;
    }
}
