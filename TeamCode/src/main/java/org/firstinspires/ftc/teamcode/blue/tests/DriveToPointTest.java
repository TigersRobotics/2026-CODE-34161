package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.controls.Controls;
import org.firstinspires.ftc.teamcode.blue.drive.DriveToPoint;
import org.firstinspires.ftc.teamcode.blue.drive.Odometry;
import org.firstinspires.ftc.teamcode.blue.drive.Pose;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;

@TeleOp(name = "Drive To Point Test", group = "Blue Test")
public class DriveToPointTest extends OpMode {
    SwerveDrive drive;
    Odometry odometry;
    DriveToPoint mover;
    DataLogger logger;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime moveTimer = new ElapsedTime();

    Pose[] points = {
            Pose.degrees(0, 0, 0),
            Pose.degrees(24, 0, 0),
            Pose.degrees(24, 24, 90),
            Pose.degrees(0, 24, 180),
            Pose.degrees(48, 0, 0)
    };
    int selected = 1;
    boolean moving = false;
    boolean arrived = false;
    double lastTime = 0;

    @Override
    public void init() {
        drive = new SwerveDrive(hardwareMap);
        odometry = new Odometry(hardwareMap);
        drive.useOdometry(odometry);
        mover = new DriveToPoint(drive, odometry);
        logger = new DataLogger("drive_to_point", "time", "targetX", "targetY", "targetH", "x", "y", "h", "distLeft", "moveP", "headingP");
    }

    @Override
    public void start() {
        timer.reset();
        odometry.setPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        odometry.update();

        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % points.length;
        if (gamepad1.dpadLeftWasPressed()) selected = (selected + points.length - 1) % points.length;
        if (gamepad1.dpadUpWasPressed()) BlueConstants.MOVE_P += 0.01;
        if (gamepad1.dpadDownWasPressed()) BlueConstants.MOVE_P = Math.max(0, BlueConstants.MOVE_P - 0.01);
        if (gamepad1.yWasPressed()) BlueConstants.HEADING_P += 0.1;
        if (gamepad1.xWasPressed()) BlueConstants.HEADING_P = Math.max(0, BlueConstants.HEADING_P - 0.1);

        if (gamepad1.aWasPressed()) {
            mover.setTarget(points[selected]);
            moving = true;
            arrived = false;
            moveTimer.reset();
        }
        if (gamepad1.bWasPressed()) moving = false;

        if (moving) {
            if (mover.update(BlueConstants.AUTO_SPEED)) {
                moving = false;
                arrived = true;
                lastTime = moveTimer.seconds();
            }
            Pose t = mover.getTarget();
            Pose p = odometry.getPose();
            logger.log(String.format("%.3f", timer.seconds()),
                    String.format("%.1f", t.x), String.format("%.1f", t.y), String.format("%.1f", Math.toDegrees(t.heading)),
                    String.format("%.2f", p.x), String.format("%.2f", p.y), String.format("%.2f", Math.toDegrees(p.heading)),
                    String.format("%.2f", mover.getDistanceLeft()),
                    String.format("%.3f", BlueConstants.MOVE_P), String.format("%.2f", BlueConstants.HEADING_P));
        } else {
            drive.drive(Controls.deadband(-gamepad1.left_stick_y) * 0.4, Controls.deadband(-gamepad1.left_stick_x) * 0.4, Controls.deadband(-gamepad1.right_stick_x) * 0.4, true);
        }

        telemetry.addLine("dpad L/R = point, A = go, B = stop");
        telemetry.addLine("dpad U/D = move P, Y/X = heading P");
        telemetry.addData("Point", points[selected]);
        telemetry.addData("Moving", moving);
        telemetry.addData("Pose", odometry.getPose());
        telemetry.addData("Left", "%.1f in  %.1f deg", mover.getDistanceLeft(), Math.toDegrees(mover.getHeadingLeft()));
        telemetry.addData("Last move", arrived ? String.format("%.2f s", lastTime) : "-");
        telemetry.addData("MOVE_P", "%.3f", BlueConstants.MOVE_P);
        telemetry.addData("HEADING_P", "%.2f", BlueConstants.HEADING_P);
    }

    @Override
    public void stop() {
        drive.stop();
        logger.close();
    }
}
