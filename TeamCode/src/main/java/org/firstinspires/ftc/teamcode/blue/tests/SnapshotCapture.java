package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;

@TeleOp(name = "Snapshot Capture", group = "Blue Test")
public class SnapshotCapture extends OpMode {
    Limelight limelight;
    SwerveDrive drive;
    ElapsedTime autoTimer = new ElapsedTime();

    String[] labels = {"pollen", "red_nectar", "blue_nectar", "mixed", "flower", "cell", "empty"};
    int label = 0;
    int count = 0;
    boolean autoCapture = false;
    String lastResult = "";

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        try {
            drive = new SwerveDrive(hardwareMap);
        } catch (Exception e) {
            drive = null;
        }
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) label = (label + 1) % labels.length;
        if (gamepad1.dpadLeftWasPressed()) label = (label + labels.length - 1) % labels.length;
        if (gamepad1.yWasPressed()) autoCapture = !autoCapture;

        if (gamepad1.aWasPressed()) capture();
        if (autoCapture && autoTimer.seconds() > 0.5) capture();

        if (gamepad1.backWasPressed()) {
            lastResult = limelight.clearSnapshots() ? "deleted all" : "delete failed";
            count = 0;
        }

        if (drive != null) {
            drive.drive(-gamepad1.left_stick_y * 0.4, -gamepad1.left_stick_x * 0.4, -gamepad1.right_stick_x * 0.4, false);
        }

        telemetry.addLine("dpad L/R = label, A = snap, Y = auto snap, BACK = delete all");
        telemetry.addData("Label", labels[label]);
        telemetry.addData("Auto", autoCapture);
        telemetry.addData("Taken", count);
        telemetry.addData("Last", lastResult);
        telemetry.addLine("Download: http://limelight.local:5801 -> Snapshots");
    }

    void capture() {
        autoTimer.reset();
        String name = labels[label] + "_" + System.currentTimeMillis();
        if (limelight.snapshot(name)) {
            count++;
            lastResult = name;
        } else {
            lastResult = "failed";
        }
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        limelight.stop();
    }
}
