package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.controls.Controls;
import org.firstinspires.ftc.teamcode.blue.drive.Odometry;
import org.firstinspires.ftc.teamcode.blue.drive.Pose;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;

@TeleOp(name = "Odometry Test", group = "Blue Test")
public class OdometryTest extends OpMode {
    SwerveDrive drive;
    Odometry odometry;
    DataLogger logger;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new SwerveDrive(hardwareMap);
        odometry = new Odometry(hardwareMap);
        drive.useOdometry(odometry);
        logger = new DataLogger("odometry", "time", "x", "y", "heading", "encX", "encY", "speed");
    }

    @Override
    public void init_loop() {
        odometry.update();
        telemetry.addData("Status", odometry.getStatus());
        telemetry.addData("Pose", odometry.getPose());
    }

    @Override
    public void start() {
        timer.reset();
        odometry.setPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        odometry.update();

        if (gamepad1.aWasPressed()) odometry.setPose(new Pose(0, 0, 0));

        drive.drive(Controls.deadband(-gamepad1.left_stick_y) * 0.5, Controls.deadband(-gamepad1.left_stick_x) * 0.5, Controls.deadband(-gamepad1.right_stick_x) * 0.5, true);

        Pose p = odometry.getPose();
        logger.log(String.format("%.3f", timer.seconds()), String.format("%.2f", p.x), String.format("%.2f", p.y),
                String.format("%.2f", Math.toDegrees(p.heading)), odometry.getEncoderX(), odometry.getEncoderY(),
                String.format("%.2f", odometry.getSpeed()));

        telemetry.addLine("Push forward 48 in -> x should read +48");
        telemetry.addLine("Push left 48 in -> y should read +48");
        telemetry.addLine("Spin 10 times -> heading should come back to 0");
        telemetry.addLine("A = reset to 0,0,0");
        telemetry.addData("Status", odometry.getStatus());
        telemetry.addData("Pose", p);
        telemetry.addData("Encoders", "%d  %d", odometry.getEncoderX(), odometry.getEncoderY());
        telemetry.addData("Log", logger.getPath());
    }

    @Override
    public void stop() {
        drive.stop();
        logger.close();
    }
}
