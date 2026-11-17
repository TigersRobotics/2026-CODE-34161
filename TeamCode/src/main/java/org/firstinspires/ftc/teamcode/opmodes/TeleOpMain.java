package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.ShotCalculator;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

import java.io.File;

import java.util.List;

@TeleOp
public class TeleOpMain extends OpMode {
    final boolean ENABLE_TURRET_CONTROLLER = true;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;
    final boolean ENABLE_WEBCAM = false;
    /// Degrees the turret moves for one tap of dpad left/right
    static final double YAW_NUDGE_DEG = 1.0;
    /// How long dpad left/right has to be held before the turret starts sweeping
    static final double YAW_HOLD_DELAY_MS = 250;
    /// Sweep speed while dpad left/right is held (0.03 = 30 degrees per second)
    static final double YAW_HOLD_DEG_PER_MS = 0.03;

    boolean enableIntake = false;
    double yaw = 0;
    double dpadRightHeldMs = 0;
    double dpadLeftHeldMs = 0;
    boolean toggleFlywheel = false;
    double targetVel = 1200;

    GoBildaOdometry odometry;
    int hiveIndex = 1;
    List<LynxModule> hubs;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        // Each hub now answers every encoder/velocity read in a loop from one bulk read,
        // instead of one hub round trip per read. The cache is cleared at the top of loop().
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER);
        ShotCalculator.loadTuning(new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt"));
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeController.init(hardwareMap, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, ENABLE_INTAKE_CONTROLLER);

        odometry = new GoBildaOdometry(hardwareMap, ODOMETRY_NAME);

        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, TuningConstants.FLYWHEEL_F_VALUE);

        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        for(LynxModule hub : hubs) hub.clearBulkCache();

        odometry.update();

        // One pose object per loop instead of a new one for each of x, y and heading
        Pose2D pose = odometry.getPose();
        double posx = pose.getX(DistanceUnit.INCH);
        double posY = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double calculatedYaw = calculateTurretAngle(hiveIndex, posx, posY, heading);
        double calculatedYawDeg = Math.toDegrees(calculatedYaw);

        double calculatedHoodAngle = calculateHoodAngle(hiveIndex, posx, posY, PollenTurretController.getVelocity());

        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        double intakePower = enableIntake ? 1 : 0;
        Timer.update();
        if(gamepad1.circleWasPressed()) toggleFlywheel = !toggleFlywheel;
        if(gamepad1.dpadUpWasPressed()) targetVel += 10;
        if(gamepad1.dpadDownWasPressed()) targetVel -= 10;
        if(gamepad1.triangleWasPressed()) hiveIndex = (hiveIndex + 1) % 2;


        TimedTelemetry.addData("Target Velocity: ", targetVel);

        PollenTurretController.setTargetVel(toggleFlywheel ? targetVel : 0);
        if(!Double.isNaN(calculatedYawDeg)) PollenTurretController.setHoodAngle(calculatedHoodAngle);

        // Manual turret offset. A tap is a fixed small step, and it uses the buffered button press
        // so a tap shorter than one loop is not lost. Holding the button sweeps after a short delay.
        if(gamepad1.dpadRightWasPressed()) yaw += YAW_NUDGE_DEG;
        if(gamepad1.dpadLeftWasPressed()) yaw -= YAW_NUDGE_DEG;

        dpadRightHeldMs = gamepad1.dpad_right ? dpadRightHeldMs + Timer.DTime() : 0;
        dpadLeftHeldMs = gamepad1.dpad_left ? dpadLeftHeldMs + Timer.DTime() : 0;
        if(dpadRightHeldMs > YAW_HOLD_DELAY_MS) yaw += Timer.DTime() * YAW_HOLD_DEG_PER_MS;
        if(dpadLeftHeldMs > YAW_HOLD_DELAY_MS) yaw -= Timer.DTime() * YAW_HOLD_DEG_PER_MS;

        // Wrapping the offset stops it winding up past a full turn, and wrapping the final angle
        // lets the turret carry on round instead of sticking at +/-180
        yaw = wrapDegrees(yaw);
        double realYaw = wrapDegrees(calculatedYawDeg - yaw);

        PollenTurretController.setYaw(realYaw);

        TimedTelemetry.addData("GoBilda Heading ", heading);
        TimedTelemetry.addData("Target Yaw: ", realYaw);

        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0));
        PollenTurretController.update();
        UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);
        SingleIntakeController.update(intakePower);
        TimedTelemetry.update();

        telemetry.update();
    }


    /**
     * Returns the angle in radians that the turret would need to be to look at the hive
     * @param hiveIndex
     * @param posX
     * @param posY
     * @param heading
     * @return
     */
    public double calculateTurretAngle(int hiveIndex, double posX, double posY, double heading) {
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


    /// Brings any angle in degrees into the range -180 to 180
    private static double wrapDegrees(double degrees) {
        degrees %= 360;
        if(degrees > 180) degrees -= 360;
        if(degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Use input inches
     */
    public double calculateHoodAngle(int hiveIndex, double posX, double posY, double flywheelTicksPerSecond) {
        final double G = 9.81;

        double hiveX = MapSpecs.HIVE_COORDINATES[hiveIndex].getX();
        double hiveY = MapSpecs.HIVE_COORDINATES[hiveIndex].getY();

        // horizontal distance to hive, inches -> meters
        double x = Mahttps://github.com/TigersRobotics/2026-CODE-34161/pull/1/conflict?name=TeamCode%252Fsrc%252Fmain%252Fjava%252Forg%252Ffirstinspires%252Fftc%252Fteamcode%252FrobotControllers%252FPollenTurretController.java&ancestor_oid=714e53535dceeac20f75212fe6501165ccf07b6a&base_oid=e4983539c38794000c5a1293293fe7fac31e4b2d&head_oid=51d9770f47b89d46ecfd1d9f137d5870c0a4105fth.hypot(hiveX - posX, hiveY - posY) * 0.0254;

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

        // prefer the flatter shot, fall back to the lob
        if (lowAngle < RobotSpecs.MAX_TURRET_ANGLE_UP && lowAngle > RobotSpecs.MAX_TURRET_ANGLE_DOWN) {
            return lowAngle;
        }
        if (highAngle < RobotSpecs.MAX_TURRET_ANGLE_UP && highAngle > RobotSpecs.MAX_TURRET_ANGLE_DOWN) {
            return highAngle;
        }

        TimedTelemetry.addLine("No valid angle found: " + lowAngle + " " + highAngle);
        return Double.NaN;
    }

    /**
     * Returns angle in degrees
     * @param add
     * @param x
     * @param y
     * @return
     */
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

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeController.stop();
        SingleIntakeController.stop();
    }
}
