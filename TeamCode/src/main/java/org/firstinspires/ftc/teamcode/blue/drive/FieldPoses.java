package org.firstinspires.ftc.teamcode.blue.drive;

import org.firstinspires.ftc.teamcode.blue.Alliance;

public class FieldPoses {
    public static Pose START = Pose.degrees(-62, 12, 0);
    public static Pose SHOOT = Pose.degrees(-36, 12, 0);
    public static Pose PICKUP = Pose.degrees(-48, 58, 90);
    public static Pose PARK = Pose.degrees(-60, 36, 0);

    public static Pose get(Pose redPose, Alliance alliance) {
        return alliance == Alliance.RED ? redPose : redPose.mirror();
    }
}
