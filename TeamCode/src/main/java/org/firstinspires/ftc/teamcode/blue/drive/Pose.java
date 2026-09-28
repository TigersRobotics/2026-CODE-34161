package org.firstinspires.ftc.teamcode.blue.drive;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Pose {
    public final double x;
    public final double y;
    public final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public static Pose degrees(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    public Pose mirror() {
        return new Pose(x, -y, AngleUnit.normalizeRadians(-heading));
    }

    public double distanceTo(Pose other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f deg)", x, y, Math.toDegrees(heading));
    }
}
