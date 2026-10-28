package org.firstinspires.ftc.teamcode.constants;

public class RobotSpecs {
    // Everything is a temp value

    public static final double TICKS_PER_SECOND = 20;
    public static final double[] CAMERA_TURRET_OFFSET= new double[]{
            0,0
    };
    // temp values
    public static final double MAX_TURRET_ANGLE_UP = 75;
    public static final double MAX_TURRET_ANGLE_DOWN = 15;
    public static final double OFFSETS_OF_ODOMETRY_PODS_CM_X = -10;
    public static final double OFFSETS_OF_ODOMETRY_PODS_CM_Y = -10;
    public static final double UPTAKE_SERVO_UP_ANGLE=0.5;
    public static final double UPTAKE_SERVO_DEFAULT_ANGLE = 0;
    // Ball speed is currently a temp value;
    public static final double BALL_SPEED = 10;

    public static final double LENGTH_X = 0.4572;
    public static final double LENGTH_Y = 0.4572;

    public static final double HEIGHT = 0.4572;

    // until further notice, intake width should be smaller than it actually is,
    // as for now, the only program using intake width recommends leaving some same space
    public static final double INTAKE_WIDTH_SHORTENED = 0.2;
    public static final double CLEARANCE = 2;

    public static final double DIAGONAL = Math.sqrt(Math.pow(LENGTH_X, 2) + Math.pow(LENGTH_Y, 2));

    public static final double FLYWHEEL_RAIL_DEGREES_PER_MM = 360;

    public static final double SPINDEXER_RPM = 60;
    public static final double SPINDEXER_MAX_ROTATION = 360; //In degrees

    public static final double ODO_WHEELS_DIA_CM = 5.0;
    public static final double TICKS_PER_REV = 3600; //Ticks per revolution of motor
    public static final double MAX_YAW = 180;
    public static final double MIN_YAW = -180;
}
