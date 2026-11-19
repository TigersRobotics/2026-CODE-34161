package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/// Field-frame pose from the Pinpoint without using setPose (which drops Y on this robot).
/// Records the raw reading once and adds the movement since then onto a known start pose.
public class FieldOdometry {
    public static double Y_SIGN = 1; // flip to -1 if driving left makes y go down
    static final int READY_LOOPS = 5;

    private final GoBildaOdometry odometry;
    private double startX, startY, startHeading; // in, in, rad
    private double rawX0, rawY0, rawH0; // in, in, rad
    private boolean pending;
    private int readyCount;

    private double x, y, heading; // in, in, rad

    public FieldOdometry(GoBildaOdometry odometry) {
        this.odometry = odometry;
    }

    public void zero(double xIn, double yIn, double headingDeg) {
        startX = xIn;
        startY = yIn;
        startHeading = Math.toRadians(headingDeg);
        pending = true;
        readyCount = 0;
    }

    public void update() {
        odometry.update();
        Pose2D raw = odometry.getPose();
        double rx = raw.getX(DistanceUnit.INCH);
        double ry = Y_SIGN * raw.getY(DistanceUnit.INCH);
        double rh = raw.getHeading(AngleUnit.RADIANS);

        if (pending) {
            if (odometry.getStatus() == GoBildaPinpointDriver.DeviceStatus.NOT_READY || ++readyCount < READY_LOOPS) {
                x = startX;
                y = startY;
                heading = startHeading;
                return;
            }
            rawX0 = rx;
            rawY0 = ry;
            rawH0 = rh;
            pending = false;
        }

        double dx = rx - rawX0;
        double dy = ry - rawY0;
        double a = startHeading - rawH0;
        x = startX + dx * Math.cos(a) - dy * Math.sin(a);
        y = startY + dx * Math.sin(a) + dy * Math.cos(a);
        heading = startHeading + (rh - rawH0);
    }
    public void reset() {

    }

    public double getX() { return x; } // in
    public double getY() { return y; } // in
    public double getHeading() { return heading; } // rad
    public boolean isZeroed() { return !pending; }
}
