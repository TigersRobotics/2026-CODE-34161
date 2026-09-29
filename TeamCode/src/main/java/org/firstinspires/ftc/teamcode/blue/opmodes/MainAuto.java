package org.firstinspires.ftc.teamcode.blue.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.Robot;
import org.firstinspires.ftc.teamcode.blue.drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.blue.drive.FieldPoses;
import org.firstinspires.ftc.teamcode.blue.drive.Pose;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.PoseStorage;
import org.firstinspires.ftc.teamcode.blue.vision.Cell;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

@Autonomous(name = "Blue Auto", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class MainAuto extends LinearOpMode {
    Robot robot;
    DriveToPoint mover;
    ElapsedTime matchTimer = new ElapsedTime();

    Alliance alliance = Alliance.BLUE;
    boolean secondStart = false;
    double delay = 0;
    int cycles = 1;
    int shotsTaken = 0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, true);
        mover = new DriveToPoint(robot.drive, robot.odometry);

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (gamepad1.yWasPressed()) secondStart = !secondStart;
            if (gamepad1.dpadUpWasPressed()) delay += 0.5;
            if (gamepad1.dpadDownWasPressed()) delay = Math.max(0, delay - 0.5);
            if (gamepad1.dpadRightWasPressed()) cycles = Math.min(3, cycles + 1);
            if (gamepad1.dpadLeftWasPressed()) cycles = Math.max(0, cycles - 1);

            robot.update();

            telemetry.addLine("X = blue, B = red, Y = start spot");
            telemetry.addLine("dpad U/D = delay, dpad L/R = cycles");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Start", secondStart ? "2" : "1");
            telemetry.addData("Delay", delay);
            telemetry.addData("Extra cycles", cycles);
            telemetry.addData("Sees cell", robot.limelight.getCell(alliance) != null);
            telemetry.addData("Start pose", startPose());
            telemetry.addData("Battery", "%.2f V%s", robot.getBattery(), robot.getBattery() < BlueConstants.LOW_BATTERY ? "  LOW" : "");
            telemetry.update();
        }

        if (isStopRequested()) return;

        matchTimer.reset();
        PoseStorage.alliance = alliance;
        robot.odometry.setPose(startPose());
        waitFor(delay);

        robot.launcher.setRpm(3000);
        goTo(pose(FieldPoses.SHOOT), 3);
        aim(1.5);
        shoot(3);

        for (int i = 0; i < cycles && timeLeft() > 9; i++) {
            robot.launcher.idle();
            goTo(pose(FieldPoses.PICKUP), 3);
            chasePollen(2);
            robot.launcher.setRpm(3000);
            goTo(pose(FieldPoses.SHOOT), 3);
            aim(1);
            shoot(2.5);
        }

        robot.launcher.stop();
        robot.intake.stop();
        goTo(pose(FieldPoses.PARK), Math.max(0.5, timeLeft() - 0.5));

        robot.update();
        PoseStorage.lastPose = robot.getPose();
        robot.stop();
    }

    Pose startPose() {
        return pose(secondStart ? FieldPoses.START_2 : FieldPoses.START);
    }

    Pose pose(Pose redPose) {
        return FieldPoses.get(redPose, alliance);
    }

    double timeLeft() {
        return 30 - matchTimer.seconds();
    }

    void status(String step) {
        PoseStorage.lastPose = robot.getPose();
        telemetry.addData("Step", step);
        telemetry.addData("Time left", "%.1f", timeLeft());
        telemetry.addData("Pose", robot.getPose());
        telemetry.addData("Shots", shotsTaken);
        telemetry.addData("Loop", "%.0f ms", robot.getLoopMs());
        telemetry.update();
    }

    void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.update();
            robot.drive.move(0, 0, 0);
            status("waiting");
        }
    }

    void goTo(Pose target, double timeout) {
        mover.setTarget(target);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeout) {
            robot.update();
            if (mover.update(BlueConstants.AUTO_SPEED)) break;
            status("going to " + target);
        }
        robot.drive.move(0, 0, 0);
    }

    void aim(double seconds) {
        robot.limelight.setPipeline(Limelight.TAG_PIPELINE);
        Pose hive = pose(FieldPoses.HIVE);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.update();
            Cell cell = robot.limelight.getCell(alliance);
            Pose p = robot.getPose();

            if (cell != null) {
                robot.launcher.aimFor(robot.limelight.getDistance(cell));
                if (Math.abs(cell.tx) < BlueConstants.AIM_TOLERANCE) break;
                robot.drive.move(0, 0, -BlueConstants.AIM_P * cell.tx);
            } else {
                robot.launcher.aimFor(p.distanceTo(hive));
                double error = AngleUnit.normalizeRadians(FieldPoses.headingTo(p, hive) - p.heading);
                if (Math.abs(Math.toDegrees(error)) < BlueConstants.AIM_TOLERANCE) break;
                robot.drive.move(0, 0, Range.clip(BlueConstants.HEADING_P * error, -0.5, 0.5));
            }
            status(cell != null ? "aiming (tags)" : "aiming (odometry)");
        }
        robot.drive.move(0, 0, 0);
    }

    void shoot(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean wasReady = false;

        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.update();
            robot.drive.move(0, 0, 0);

            boolean ready = robot.launcher.isReady();
            if (ready) {
                robot.launcher.feed();
                robot.intake.in();
            } else {
                robot.launcher.stopFeed();
                robot.intake.stop();
                if (wasReady) shotsTaken++;
            }
            wasReady = ready;
            status("shooting");
        }
        robot.launcher.stopFeed();
        robot.intake.stop();
    }

    void chasePollen(double seconds) {
        robot.limelight.setPipeline(Limelight.DETECTOR_PIPELINE);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.update();
            Target pollen = robot.limelight.getClosest(Target.Type.POLLEN);
            robot.intake.in();

            if (pollen == null) {
                robot.drive.move(0, 0, 0.3);
            } else {
                robot.drive.move(0.4, 0, -BlueConstants.PICKUP_P * pollen.tx);
            }
            status("chasing pollen");
        }
        robot.drive.move(0, 0, 0);
        robot.limelight.setPipeline(Limelight.TAG_PIPELINE);
    }
}
