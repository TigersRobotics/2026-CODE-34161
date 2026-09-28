package org.firstinspires.ftc.teamcode.blue.drive;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.PID;

public class DriveToPoint {
    private final SwerveDrive drive;
    private final Odometry odometry;

    private final PID xPid = new PID(BlueConstants.MOVE_P, 0, BlueConstants.MOVE_D);
    private final PID yPid = new PID(BlueConstants.MOVE_P, 0, BlueConstants.MOVE_D);
    private final PID headingPid = new PID(BlueConstants.HEADING_P, 0, BlueConstants.HEADING_D);

    private Pose target = new Pose(0, 0, 0);
    private double distanceLeft = 0;
    private double headingLeft = 0;

    public DriveToPoint(SwerveDrive drive, Odometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
    }

    public void setTarget(Pose target) {
        this.target = target;
        xPid.reset();
        yPid.reset();
        headingPid.reset();
    }

    public Pose getTarget() {
        return target;
    }

    public double getDistanceLeft() {
        return distanceLeft;
    }

    public double getHeadingLeft() {
        return headingLeft;
    }

    public boolean update(double maxSpeed) {
        xPid.kP = BlueConstants.MOVE_P;
        xPid.kD = BlueConstants.MOVE_D;
        yPid.kP = BlueConstants.MOVE_P;
        yPid.kD = BlueConstants.MOVE_D;
        headingPid.kP = BlueConstants.HEADING_P;
        headingPid.kD = BlueConstants.HEADING_D;

        Pose p = odometry.getPose();
        double ex = target.x - p.x;
        double ey = target.y - p.y;
        double eh = AngleUnit.normalizeRadians(target.heading - p.heading);

        distanceLeft = Math.hypot(ex, ey);
        headingLeft = eh;

        if (distanceLeft < BlueConstants.POSITION_TOLERANCE && Math.abs(eh) < BlueConstants.HEADING_TOLERANCE) {
            drive.move(0, 0, 0);
            return true;
        }

        double fx = xPid.calculate(ex);
        double fy = yPid.calculate(ey);
        double mag = Math.hypot(fx, fy);
        if (mag > maxSpeed) {
            fx = fx / mag * maxSpeed;
            fy = fy / mag * maxSpeed;
        }
        double turn = Range.clip(headingPid.calculate(eh), -maxSpeed, maxSpeed);

        double h = -p.heading;
        double forward = fx * Math.cos(h) - fy * Math.sin(h);
        double left = fx * Math.sin(h) + fy * Math.cos(h);

        drive.move(forward, left, turn);
        return false;
    }
}
