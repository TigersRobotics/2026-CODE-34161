package org.firstinspires.ftc.teamcode.blue.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class BlueConstants {
    public static double TRACK_WIDTH = 12.0;
    public static double TRACK_LENGTH = 12.0;

    public static double FL_OFFSET = 0;
    public static double FR_OFFSET = 0;
    public static double BL_OFFSET = 0;
    public static double BR_OFFSET = 0;

    public static boolean FL_REVERSED = false;
    public static boolean FR_REVERSED = false;
    public static boolean BL_REVERSED = false;
    public static boolean BR_REVERSED = false;
    public static boolean TURN_REVERSED = false;

    public static double TURN_P = 0.6;
    public static double TURN_I = 0;
    public static double TURN_D = 0.01;
    public static double TURN_TOLERANCE = Math.toRadians(1.5);

    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static double DEADBAND = 0.05;
    public static double SLOW_SPEED = 0.35;

    public static double CAMERA_HEIGHT = 10.0;
    public static double CAMERA_PITCH = 25.0;
    public static double TAG_HEIGHT = 46.0;
    public static double AIM_P = 0.025;
    public static double AIM_TOLERANCE = 1.5;
    public static double PICKUP_P = 0.02;
    public static double MIN_CONFIDENCE = 0.5;

    public static double LAUNCHER_TICKS_PER_REV = 28;
    public static double LAUNCHER_RPM_TOLERANCE = 100;
    public static double LAUNCHER_IDLE_RPM = 0;
    public static double HOOD_MIN = 0.1;
    public static double HOOD_MAX = 0.9;
    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 1.0;
}
