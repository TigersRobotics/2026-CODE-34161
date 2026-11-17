package org.firstinspires.ftc.teamcode.constants;

public class RobotSpecs {
    // Everything is a temp value

    public static final double[] CAMERA_TURRET_OFFSET= new double[]{
            0,0
    };
    // temp values

    public static final double MS_PER_PATH_UPDATE = 500;
    public static final double MAX_TURRET_ANGLE_UP = 25;
    public static final double MAX_TURRET_ANGLE_DOWN = 5;
    public static final double OFFSETS_OF_ODOMETRY_PODS_IN_X = -3.3948507834607224;
    public static final double OFFSETS_OF_ODOMETRY_PODS_IN_Y = -5.696962611881767;
    public static final double UPTAKE_SERVO_UP_ANGLE=0.5;
    public static final double UPTAKE_SERVO_DEFAULT_ANGLE = 0;
    // Ball speed is currently a temp value;
    public static final double BALL_SPEED = 10;

    public static final double LENGTH_X_M = 0.4572;
    public static final double HEIGHT_M = 0.4572;

    // until further notice, intake width should be smaller than it actually is,
    // as for now, the only program using intake width recommends leaving some same space
    public static final double INTAKE_WIDTH_SHORTENED_CM = 33;
    public static final double CLEARANCE_IN = 0.5;
    public static final double FLYWHEEL_RAIL_DEGREES_PER_MM = 360;

    public static final double SPINDEXER_RPM = 60;
    public static final double SPINDEXER_MAX_ROTATION = 360; //In degrees

    public static final double ODO_WHEELS_DIA_CM = 5.0;
    public static final double TICKS_PER_REV = 700; //Ticks per revolution of motor
    public static final double MAX_YAW = 180;
    public static final double MIN_YAW = -180;
    public static final double HOOD_HEIGHT = 10; ///Inches
}
