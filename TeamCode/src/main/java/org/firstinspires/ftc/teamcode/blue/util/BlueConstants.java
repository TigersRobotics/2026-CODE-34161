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
    public static boolean ENCODER_REVERSED = false;

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
    public static long MAX_STALE_MS = 100;

    public static double LAUNCHER_TICKS_PER_REV = 28;
    public static double LAUNCHER_RPM_TOLERANCE = 100;
    public static double LAUNCHER_IDLE_RPM = 0;
    public static double HOOD_MIN = 0.1;
    public static double HOOD_MAX = 0.9;
    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 1.0;

    public static double PINPOINT_X_OFFSET = 0;
    public static double PINPOINT_Y_OFFSET = 0;
    public static boolean PINPOINT_X_REVERSED = false;
    public static boolean PINPOINT_Y_REVERSED = false;

    public static double MOVE_P = 0.06;
    public static double MOVE_D = 0.004;
    public static double HEADING_P = 1.2;
    public static double HEADING_D = 0.02;
    public static double POSITION_TOLERANCE = 1.0;
    public static double HEADING_TOLERANCE = Math.toRadians(3);
    public static double AUTO_SPEED = 0.7;

    public static int LIFT_DOWN = 0;
    public static int LIFT_FLOWER = 1500;
    public static int LIFT_MAX = 1800;
    public static double LIFT_POWER = 1.0;

    public static float COLOR_GAIN = 8;
    public static double BALL_DISTANCE = 4.0;
    public static double YELLOW_MIN = 40;
    public static double YELLOW_MAX = 90;
    public static double BLUE_MIN = 180;
    public static double BLUE_MAX = 260;
    public static double RED_LOW = 25;
    public static double RED_HIGH = 330;

    public static double TELEOP_LENGTH = 120;

    public static double LAUNCHER_P = 1.2;
    public static double LAUNCHER_I = 0.12;
    public static double LAUNCHER_D = 0;
    public static double LAUNCHER_F = 11.7;

    public static double HOLD_P = 0.8;
    public static double HOLD_D = 0.02;
    public static double HOLD_DELAY = 0.25;
    public static double HOLD_MAX = 0.5;

    public static double LOW_BATTERY = 12.3;
}
