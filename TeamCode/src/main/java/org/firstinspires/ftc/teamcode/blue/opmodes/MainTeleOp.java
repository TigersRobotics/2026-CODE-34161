package org.firstinspires.ftc.teamcode.blue.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.Robot;
import org.firstinspires.ftc.teamcode.blue.controls.Controls;
import org.firstinspires.ftc.teamcode.blue.drive.FieldPoses;
import org.firstinspires.ftc.teamcode.blue.drive.HeadingHold;
import org.firstinspires.ftc.teamcode.blue.drive.Pose;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.MatchTimer;
import org.firstinspires.ftc.teamcode.blue.util.PoseStorage;
import org.firstinspires.ftc.teamcode.blue.vision.Cell;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

@TeleOp(name = "Blue TeleOp", group = "Blue")
public class MainTeleOp extends OpMode {
    Robot robot;
    Controls controls;
    HeadingHold hold = new HeadingHold();
    MatchTimer matchTimer = new MatchTimer();

    Alliance alliance = PoseStorage.alliance;
    boolean fieldCentric = true;
    boolean launcherOn = false;
    boolean autoDistance = true;
    boolean liftManual = false;
    double manualRpm = 3000;
    double manualHood = 0.5;
    Target.Type lastBall = null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        controls = new Controls(gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
        robot.update();
        if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = Alliance.RED;
        if (gamepad1.yWasPressed()) hold.enabled = !hold.enabled;

        telemetry.addLine("X = blue, B = red, Y = heading hold");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Heading hold", hold.enabled);
        telemetry.addData("Limelight", robot.limelight.isConnected() ? "connected" : "NOT CONNECTED");
        telemetry.addData("Odometry", robot.odometry == null ? "not found" : "ok");
        telemetry.addData("Pose from auto", PoseStorage.lastPose == null ? "none" : PoseStorage.lastPose);
        telemetry.addData("Ball sensor", robot.ballSensor == null ? "not found" : "ok");
        telemetry.addData("Lift", robot.lift == null ? "not found" : "ok");
        telemetry.addData("Battery", "%.2f V%s", robot.getBattery(), robot.getBattery() < BlueConstants.LOW_BATTERY ? "  LOW" : "");
    }

    @Override
    public void start() {
        robot.limelight.setPipeline(Limelight.TAG_PIPELINE);
        controls.clearPresses();
        matchTimer.start();

        if (robot.odometry != null && PoseStorage.lastPose != null) {
            robot.odometry.setPose(PoseStorage.lastPose);
        }
    }

    @Override
    public void loop() {
        robot.update();
        matchTimer.update(gamepad1, gamepad2);

        if (controls.resetHeading()) {
            robot.drive.resetHeading();
            hold.release();
        }
        if (controls.toggleFieldCentric()) fieldCentric = !fieldCentric;

        double speed = controls.slow() ? BlueConstants.SLOW_SPEED : 1.0;
        double forward = controls.forward() * speed;
        double strafe = controls.strafe() * speed;
        boolean translating = Math.hypot(forward, strafe) > BlueConstants.DEADBAND;
        double turn = hold.update(controls.turn() * speed, robot.drive.getHeading(), translating);

        Pose pose = robot.getPose();
        Pose hive = FieldPoses.get(FieldPoses.HIVE, alliance);
        Cell cell = robot.limelight.getCell(alliance);

        double distance = -1;
        if (cell != null) {
            distance = robot.limelight.getDistance(cell);
        } else if (pose != null) {
            distance = pose.distanceTo(hive);
        }

        boolean aimed = false;
        boolean chasing = false;

        if (controls.autoAim()) {
            hold.release();
            robot.limelight.setPipeline(Limelight.TAG_PIPELINE);
            if (cell != null) {
                turn = -BlueConstants.AIM_P * cell.tx;
                aimed = Math.abs(cell.tx) < BlueConstants.AIM_TOLERANCE;
            } else if (pose != null) {
                double error = AngleUnit.normalizeRadians(FieldPoses.headingTo(pose, hive) - pose.heading);
                turn = Range.clip(BlueConstants.HEADING_P * error, -0.6, 0.6);
                aimed = Math.abs(Math.toDegrees(error)) < BlueConstants.AIM_TOLERANCE;
            }
        } else if (controls.chasePollen()) {
            hold.release();
            robot.limelight.setPipeline(Limelight.DETECTOR_PIPELINE);
            Target pollen = robot.limelight.getClosest(Target.Type.POLLEN);
            if (pollen != null) {
                turn = -BlueConstants.PICKUP_P * pollen.tx;
                chasing = true;
            }
        } else {
            robot.limelight.setPipeline(Limelight.TAG_PIPELINE);
        }

        if (controls.lockWheels()) {
            robot.drive.lock();
        } else {
            robot.drive.drive(forward, strafe, turn, fieldCentric);
        }

        if (controls.toggleLauncher()) launcherOn = !launcherOn;
        if (controls.toggleAutoDistance()) autoDistance = !autoDistance;

        if (controls.rpmUp()) manualRpm = Math.min(6000, manualRpm + 100);
        if (controls.rpmDown()) manualRpm = Math.max(500, manualRpm - 100);
        if (controls.hoodUp()) {
            manualHood = Math.min(BlueConstants.HOOD_MAX, manualHood + 0.05);
            autoDistance = false;
        }
        if (controls.hoodDown()) {
            manualHood = Math.max(BlueConstants.HOOD_MIN, manualHood - 0.05);
            autoDistance = false;
        }

        if (!launcherOn) {
            robot.launcher.stop();
        } else if (autoDistance && distance > 0) {
            robot.launcher.aimFor(distance);
        } else {
            robot.launcher.setRpm(manualRpm);
            robot.launcher.setHood(manualHood);
        }

        boolean shooting = controls.shoot() && robot.launcher.isReady();

        if (shooting) {
            robot.launcher.feed();
        } else if (controls.unjam()) {
            robot.launcher.unjam();
        } else {
            robot.launcher.stopFeed();
        }

        if (controls.intakeOut()) {
            robot.intake.out();
        } else if (controls.intakeIn() || chasing || shooting) {
            robot.intake.in();
        } else {
            robot.intake.stop();
        }

        if (robot.lift != null) {
            double liftPower = controls.liftManual();
            if (controls.liftFlower()) {
                robot.lift.flower();
                liftManual = false;
            } else if (controls.liftDown()) {
                robot.lift.down();
                liftManual = false;
            } else if (liftPower != 0) {
                robot.lift.manual(liftPower);
                liftManual = true;
            } else if (liftManual) {
                robot.lift.holdHere();
                liftManual = false;
            }
        }

        if (robot.ballSensor != null) {
            Target.Type ball = robot.ballSensor.getBall();
            if (ball != null && ball != lastBall) controls.rumbleOperator(150);
            lastBall = ball;
        }

        telemetry.addData("Time", matchTimer.display());
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive", "%s%s", fieldCentric ? "field" : "robot", hold.isHolding() ? "  holding" : "");
        telemetry.addData("Heading", "%.1f", robot.drive.getHeadingDegrees());
        if (pose != null) telemetry.addData("Pose", pose);
        telemetry.addData("Cell", cell == null ? "none" : cell);
        telemetry.addData("Distance", "%.1f%s", distance, cell == null && distance > 0 ? " (odo)" : "");
        telemetry.addData("Aimed", aimed);
        telemetry.addData("Launcher", "%s  %.0f / %.0f rpm  hood %.2f", launcherOn ? "ON" : "off",
                robot.launcher.getRpm(), robot.launcher.getTargetRpm(), robot.launcher.getHood());
        telemetry.addData("Auto distance", autoDistance);
        telemetry.addData("Ready", robot.launcher.isReady());
        if (robot.ballSensor != null) telemetry.addData("Loaded", lastBall == null ? "nothing" : lastBall);
        if (robot.lift != null) telemetry.addData("Lift", "%d / %d", robot.lift.getPosition(), robot.lift.getTarget());
        telemetry.addData("Loop", "%.0f ms   battery %.1f V", robot.getLoopMs(), robot.getBattery());
    }

    @Override
    public void stop() {
        robot.stop();
        PoseStorage.lastPose = null;
    }
}
