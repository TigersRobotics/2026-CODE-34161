package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

public class VectorMath {

    public static double[] rotateVector(double x, double y, double angleInRadians) {
        double cos = Math.cos(angleInRadians);
        double sin = Math.sin(angleInRadians);

        double newX = x * cos - y * sin;
        double newY = x * sin + y * cos;

        return new double[]{newX, newY};
    }

    public static double[] rotateVector(double x, double y, double angle, boolean inRad) {
        if (inRad) {
            return rotateVector(x, y, angle);
        } else {
            return rotateVector(x, y, Math.toRadians(angle));
        }
    }
    

    public static double[] shotAngle(double d, double h, double r, double camAngle) {
/// returns turret vertical angle, and the horizonal angle. Input angleFromTag: facing right is pos
/// all angles are in degrees, lengths are in meters, speeds are in m/s
        double [] nV = addV(new double[]{d, camAngle}, new double[]{MapSpecs.APRIL_TAG_HIVE_OFFSET_D, 180-r});
        double y = h + MapSpecs.APRIL_TAG_HIVE_OFFSET_H;
        double x = nV[0];

        // it will try to calculate the angle for trajectory
        double angle = angleCalc((byte) -1, x, y);
        double nAngle;

        // checks if the angle is valid
        if (angle >= RobotSpecs.MAX_TURRET_ANGLE_UP ||
                angle <= RobotSpecs.MAX_TURRET_ANGLE_DOWN || Double.isNaN(angle)){

            // invalid initial angle will return a second angle
            nAngle = angleCalc((byte) 1, x, y);
            if (nAngle >= RobotSpecs.MAX_TURRET_ANGLE_UP ||
                    nAngle <= RobotSpecs.MAX_TURRET_ANGLE_DOWN || Double.isNaN(nAngle)){

                // will log that it cannot find valid angle, shows init angle and backup angle
                TimedTelemetry.addLine(("No valid angle found: " + angle + " " + nAngle));
                return new double[] {Double.NaN};
            }
            return new double[] {nAngle, nV[1]};
        }

        return new double[] {angle, nV[1]};
    }

    private static double angleCalc(byte add, double x, double y) {
        double g = 9.800665;

        double v = RobotSpecs.BALL_SPEED;
        double v2 = Math.pow(v, 2);
        double v4 = Math.pow(v, 4);

        double x2 = Math.pow(x, 2);
        double discriminant = v4 - g * (g * x2 + 2 * y * v2);
        if (discriminant < 0) {
            return Double.NaN; // Target is physically out of reach
        }

        double root = add * Math.sqrt(discriminant);
        double arg = (v2 + root) / (g * x);

        return Math.atan(arg)/Math.PI*180;
    }

    private static double[] addV(double[] v1, double[] v2){
        double r1 = v1[0];
        double t1 = v1[1]*Math.PI/180;

        double r2 = v2[0];
        double t2 = v2[1]*Math.PI/180;

        double x1 = r1*Math.cos(t1);
        double y1 = r1*Math.sin(t1);

        double x2 = r2*Math.cos(t2);
        double y2 = r2*Math.sin(t2);

        x1+=x2;
        y1+=y2;

        r1 = Math.sqrt((Math.pow(x1, 2) + Math.pow(y1, 2)));
        t1 = Math.atan2(y1, x1)*180/Math.PI;

        return new double[] {r1,t1};

    }



    /**
     * Returns the angle in radians that the turret would need to be to look at the hive
     * @param hiveIndex the hive that the robot is looking at(0 is left, 1 is right)
     * @param posX  Robot x pos
     * @param posY  robot y pos
     * @param heading robot heading
     * @return returns a yaw for the turret to be at in radians
     */
    public static double calculateTurretAngle(int hiveIndex, double posX, double posY, double heading) {
        double hiveX = MapSpecs.HIVE_COORDINATES[hiveIndex].getX();
        double hiveY = MapSpecs.HIVE_COORDINATES[hiveIndex].getY();

        double dx = hiveX - posX;
        double dy = hiveY - posY;

        double calculatedYaw = Math.atan2(dy, dx);
        calculatedYaw += heading;


        while(calculatedYaw > Math.PI) calculatedYaw -= 2 * Math.PI;
        while(calculatedYaw < -Math.PI) calculatedYaw += 2* Math.PI;

        return calculatedYaw;
    }


    /**
     * Use input inches
     */
    public static double calculateHoodAngle(int hiveIndex, double posX, double posY, double flywheelTicksPerSecond) {
        final double G = 9.81;

        double hiveX = MapSpecs.HIVE_COORDINATES[hiveIndex].getX();
        double hiveY = MapSpecs.HIVE_COORDINATES[hiveIndex].getY();

        // horizontal distance to hive, inches -> meters
        double x = Math.hypot(hiveX - posX, hiveY - posY) * 0.0254;

        // target height relative to where the ball leaves the shooter (placeholders), inches -> meters
        double y = (MapSpecs.HIVE_HEIGHT - RobotSpecs.HOOD_HEIGHT) * 0.0254;

        // flywheel ticks/sec -> ball exit speed in m/s (tune the efficiency!)
        double wheelRevsPerSec = flywheelTicksPerSecond / RobotSpecs.FLYWHEEL_TICKS_PER_REV;
        double surfaceSpeed = wheelRevsPerSec * 2 * Math.PI * RobotSpecs.FLYWHEEL_RADIUS_M;
        double v = surfaceSpeed * RobotSpecs.SHOT_EFFICIENCY;

        if (v <= 0 || x <= 0) return Double.NaN;

        // Solve y = x*u - k*(1+u^2) * ... for u = tan(angle):
        // k*u^2 - x*u + (k + y) = 0, where k = g*x^2 / (2*v^2)
        double k = G * x * x / (2 * v * v);
        double disc = x * x - 4 * k * (k + y);

        if (disc < 0) {
            TimedTelemetry.addLine("Flywheel too slow to reach target");
            return Double.NaN;
        }

        double sqrtDisc = Math.sqrt(disc);
        double lowAngle  = Math.toDegrees(Math.atan((x - sqrtDisc) / (2 * k))); // flatter shot
        double highAngle = Math.toDegrees(Math.atan((x + sqrtDisc) / (2 * k))); // lobbed shot


        //Prefer not lob shot
        if (lowAngle < RobotSpecs.MAX_TURRET_ANGLE_UP && lowAngle > RobotSpecs.MAX_TURRET_ANGLE_DOWN) {
            return lowAngle;
        }
        //lob shot
        if (highAngle < RobotSpecs.MAX_TURRET_ANGLE_UP && highAngle > RobotSpecs.MAX_TURRET_ANGLE_DOWN) {
            return highAngle;
        }


        TimedTelemetry.addLine("No valid angle found: " + lowAngle + " " + highAngle);
        return Double.NaN;
    }

}