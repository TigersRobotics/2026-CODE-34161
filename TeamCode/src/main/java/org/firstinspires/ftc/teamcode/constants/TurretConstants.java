package org.firstinspires.ftc.teamcode.constants;

public class TurretConstants {
    public static final double HOOD_MIN_DEG = 5;
    public static final double HOOD_MAX_DEG = 22.5;
    public static final double HOOD_STEP_DEG = 0.5;

    public static final double FLYWHEEL_TICKS_PER_REV = 28;
    public static final double FLYWHEEL_RPM_MIN = 0;
    public static final double FLYWHEEL_RPM_MAX = 6000;

    public static final double YAW_TICKS_PER_REV = 732.2;
    public static final double YAW_RPM_MAX = 240;
    public static final double YAW_STEP_DEG = 0.5;

    // Tuned Constants

    // PIDF
    public static double FLYWHEEL_P = 10;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 32767.0 / (FLYWHEEL_RPM_MAX * FLYWHEEL_TICKS_PER_REV / 60.0);

    // Shooting
    public static double G = 9.80665; // m/s^2
    public static double DRAG_K = 0.0458; // 1/m
    public static double WHEEL_DIAMETER = 0.090; // m
    public static double HIVE_Z = 1.5129; // m
    public static double SHOOTER_Z = 0.305; // m
    public static double HIVE_X = 0; // m
    public static double HIVE_Y = 0; // m

    public static double W_V = 0.5;
    public static double W_D = 1.25;
    public static double W_H1 = -1.0;
    public static double W_H0 = 90.0;
    public static double W_BETA = 0.0;
    public static double PHI_STAR = 30.0; // deg
    public static double W_T = 1.0;
    public static double T_LAT = 0.05; // s

    public static double RPM_A = 2250;
    public static double RPM_B = 313; // rpm per m

    public static double YAW_TOL = 0.15; // m
    public static double HOOD_SETTLE_DEG = 0.5;
    public static long HOOD_SETTLE_MS = 150;
    public static double RPM_STABLE = 50;
    public static long RPM_STABLE_MS = 100;
    public static double HOOD_EDGE_DEG = 0.5;
    public static double MAX_MOVING_SPEED = 1.0; // m/s
    public static double MAX_LEAD_RATIO = 0.5;

    public static double BEHIND_HYST = 0.05; // m
    public static double BEHIND_MIN_PHI = 60; // deg

    public static double TRIM_STEP_START = 2.0; // deg
    public static double TRIM_SHORTER_SIGN = -1;
}
