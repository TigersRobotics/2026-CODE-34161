package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;

@TeleOp(name = "Launcher Test", group = "Blue Test")
public class LauncherTest extends OpMode {
    Launcher launcher;
    Intake intake;
    Limelight limelight;
    DataLogger shots;
    DataLogger rpmLog;
    ElapsedTime timer = new ElapsedTime();

    Alliance alliance = Alliance.BLUE;
    boolean on = false;
    double rpm = 3000;
    double hood = 0.5;
    int made = 0;
    int missed = 0;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);

        shots = new DataLogger("shots", "time", "distance", "tagId", "targetRpm", "actualRpm", "hood", "result");
        rpmLog = new DataLogger("launcher_rpm", "time", "targetRpm", "actualRpm", "feeding");
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = Alliance.RED;
        telemetry.addData("Alliance (X/B)", alliance);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        limelight.update(0);
        LLResultTypes.FiducialResult tag = limelight.getCellTag(alliance);
        double distance = tag == null ? -1 : limelight.getDistance(tag);

        if (gamepad1.aWasPressed()) on = !on;
        if (gamepad1.dpadUpWasPressed()) rpm += 100;
        if (gamepad1.dpadDownWasPressed()) rpm -= 100;
        if (gamepad1.dpadRightWasPressed()) hood += 0.02;
        if (gamepad1.dpadLeftWasPressed()) hood -= 0.02;

        if (on) {
            launcher.setRpm(rpm);
        } else {
            launcher.stop();
        }
        launcher.setHood(hood);
        hood = launcher.getHood();

        boolean feeding = gamepad1.right_bumper;
        if (feeding) {
            launcher.feed();
            intake.in();
        } else {
            launcher.stopFeed();
            intake.stop();
        }

        if (gamepad1.xWasPressed()) {
            made++;
            logShot(distance, tag, "made");
        }
        if (gamepad1.bWasPressed()) {
            missed++;
            logShot(distance, tag, "missed");
        }

        rpmLog.log(String.format("%.3f", timer.seconds()), String.format("%.0f", launcher.getTargetRpm()),
                String.format("%.0f", launcher.getRpm()), feeding);

        telemetry.addLine("A = flywheel, RB = feed, dpad U/D = rpm, dpad L/R = hood");
        telemetry.addLine("X = made it, B = missed");
        telemetry.addData("Flywheel", on ? "ON" : "off");
        telemetry.addData("RPM", "%.0f / %.0f", launcher.getRpm(), rpm);
        telemetry.addData("Ready", launcher.isReady());
        telemetry.addData("Hood", "%.2f", hood);
        telemetry.addData("Distance", "%.1f", distance);
        telemetry.addData("Made / missed", made + " / " + missed);
        telemetry.addData("Shots log", shots.getPath());
    }

    void logShot(double distance, LLResultTypes.FiducialResult tag, String result) {
        shots.log(String.format("%.3f", timer.seconds()), String.format("%.1f", distance),
                tag == null ? "" : tag.getFiducialId(),
                String.format("%.0f", rpm), String.format("%.0f", launcher.getRpm()),
                String.format("%.2f", hood), result);
    }

    @Override
    public void stop() {
        launcher.stop();
        intake.stop();
        limelight.stop();
        shots.close();
        rpmLog.close();
    }
}
