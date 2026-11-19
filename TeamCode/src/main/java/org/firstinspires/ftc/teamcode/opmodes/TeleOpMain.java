package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;
import static org.firstinspires.ftc.teamcode.utils.VectorMath.calculateHoodAngle;
import static org.firstinspires.ftc.teamcode.utils.VectorMath.calculateTurretAngle;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.controller.CustomController;
import org.firstinspires.ftc.teamcode.controller.Presets;
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


    double posx;
    double posY;
    double heading;
    double intakePower;

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


    private void updateCache() {
        // One pose object per loop instead of a new one for each of x, y and heading
        Pose2D pose = odometry.getPose();
        posx = pose.getX(DistanceUnit.INCH);
        posY = pose.getY(DistanceUnit.INCH);
        heading = pose.getHeading(AngleUnit.RADIANS);
    }
    @Override
    public void loop() {
        for(LynxModule hub : hubs) hub.clearBulkCache();
        Timer.update();

        handleTechnicianInputs();
        handleDriverInputs();

        odometry.update();
        updateCache();
        handleTurret();


        TimedTelemetry.addData("Target Velocity: ", targetVel);


        // Wrapping the offset stops it winding up past a full turn, and wrapping the final angle
        // lets the turret carry on round instead of sticking at +/-180


        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0));
        PollenTurretController.update();
        UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);
        SingleIntakeController.update(intakePower);
        TimedTelemetry.update();

        telemetry.update();
    }

    private void handleDriverInputs() {
        if(gamepad1.leftTriggerWasPressed()) enableIntake = !enableIntake;
        intakePower = enableIntake ? 1 : 0;
    }

    private void handleTechnicianInputs() {
        if(gamepad2.dpadUpWasPressed()) targetVel += 10;
        if(gamepad2.dpadDownWasPressed()) targetVel -= 10;
        if(gamepad1.triangleWasPressed()) hiveIndex = (hiveIndex + 1) % 2;
        if(gamepad2.circleWasPressed()) toggleFlywheel = !toggleFlywheel;



        // Manual turret offset. A tap is a fixed small step, and it uses the buffered button press
        // so a tap shorter than one loop is not lost. Holding the button sweeps after a short delay.
        if(gamepad1.dpadRightWasPressed()) yaw += YAW_NUDGE_DEG;
        if(gamepad1.dpadLeftWasPressed()) yaw -= YAW_NUDGE_DEG;

        dpadRightHeldMs = gamepad1.dpad_right ? dpadRightHeldMs + Timer.DTime() : 0;
        dpadLeftHeldMs = gamepad1.dpad_left ? dpadLeftHeldMs + Timer.DTime() : 0;
        if(dpadRightHeldMs > YAW_HOLD_DELAY_MS) yaw += Timer.DTime() * YAW_HOLD_DEG_PER_MS;
        if(dpadLeftHeldMs > YAW_HOLD_DELAY_MS) yaw -= Timer.DTime() * YAW_HOLD_DEG_PER_MS;

    }

    private void handleTurret() {
        double calculatedYaw = calculateTurretAngle(hiveIndex, posx, posY, heading);
        double calculatedYawDeg = Math.toDegrees(calculatedYaw);

        double calculatedHoodAngle = calculateHoodAngle(hiveIndex, posx, posY, PollenTurretController.getVelocity());

        PollenTurretController.setTargetVel(toggleFlywheel ? targetVel : 0);
        if(!Double.isNaN(calculatedYawDeg)) PollenTurretController.setHoodAngle(calculatedHoodAngle);

        yaw = wrapDegrees(yaw);
        double realYaw = wrapDegrees(calculatedYawDeg - yaw);
        PollenTurretController.setYaw(realYaw);

        TimedTelemetry.addData("GoBilda Heading ", heading);
        TimedTelemetry.addData("Target Yaw: ", realYaw);
    }
    /// Brings any angle in degrees into the range -180 to 180
    private static double wrapDegrees(double degrees) {
        degrees %= 360;
        if(degrees > 180) degrees -= 360;
        if(degrees < -180) degrees += 360;
        return degrees;
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeController.stop();
        SingleIntakeController.stop();
    }
}
