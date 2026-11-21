package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_UP_ANGLE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.utils.VectorMath.calculateHoodAngle;
import static org.firstinspires.ftc.teamcode.utils.VectorMath.calculateTurretAngle;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.sensors.FieldOdometry;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.ShotCalculator;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

import java.io.File;
import java.util.List;

@TeleOp
public class TeleOpMain extends OpMode {
    private boolean tipped;
    private double trim;
    final boolean ENABLE_TURRET_CONTROLLER = true;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;

    boolean enableIntake = false;
    double yaw = 0;
    boolean toggleFlywheel = false;
    double targetVel = 1320;

    FieldOdometry odometry;
    int hiveIndex = 1;
    List<LynxModule> hubs;


    double posx;
    double posY;
    double heading = 90;
    double intakePower;
    double angle = 15;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        // Each hub now answers every encoder/velocity read in a loop from one bulk read,
        // instead of one hub round trip per read. The cache is cleared at the top of loop().
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER, true);
        ShotCalculator.loadTuning(new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt"));
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeController.init(hardwareMap, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, ENABLE_INTAKE_CONTROLLER);

        hoodServo = hardwareMap.get(Servo.class, TURRET_UP_ANGLE_SERVO_NAME);
        hoodServo.setPosition(ShotCalculator.hoodServoPosition(angle));
        odometry = new FieldOdometry(new GoBildaOdometry(hardwareMap, ODOMETRY_NAME));
        odometry.zero(-72 + 6.5, -72 + 9, 90);

        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, TuningConstants.FLYWHEEL_F_VALUE);

        TimedTelemetry.update();
        telemetry.update();
    }

    Servo hoodServo;

    private void updateCache() {
        // One pose object per loop instead of a new one for each of x, y and heading
        posx = odometry.getX();
        posY = odometry.getY();
        heading = odometry.getHeading();
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


        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0), odometry.getHeading());
        PollenTurretController.update();
        UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);
        SingleIntakeController.update(intakePower);
        TimedTelemetry.update();

        telemetry.update();
    }

    private void handleDriverInputs() {
        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        intakePower = enableIntake ? 1 : 0;
    }

    private void handleTechnicianInputs() {
        if(gamepad2.dpadUpWasPressed()) targetVel += 10;
        if(gamepad2.dpadDownWasPressed()) targetVel -= 10;
        if(gamepad1.triangleWasPressed()) hiveIndex = (hiveIndex + 1) % 2;
        if(gamepad2.circleWasPressed()) toggleFlywheel = !toggleFlywheel;

        if(gamepad2.rightBumperWasPressed()) angle += 0.5;
        if(gamepad2.leftBumperWasPressed()) angle -= 0.5;

        // Manual turret offset. A tap is a fixed small step, and it uses the buffered button press
        // so a tap shorter than one loop is not lost. Holding the button sweeps after a short delay.
        yaw += gamepad2.right_stick_x * Timer.DTime() *0.25;

    }

    private void handleTurret() {

        double[] t = Field.upCell(true, tipped);
        double fieldDeg = Math.toDegrees(Math.atan2(t[1] - odometry.getY(), t[0] - odometry.getX())); // deg
        double turret = wrap(Math.toDegrees(odometry.getHeading()) - fieldDeg + trim); // deg
        if (odometry.isZeroed()) PollenTurretController.setYaw(turret + yaw);

        double calculatedYaw = calculateTurretAngle(hiveIndex, posx, posY, heading);
        double calculatedYawDeg = Math.toDegrees(calculatedYaw);

        double calculatedHoodAngle = calculateHoodAngle(hiveIndex, posx, posY, PollenTurretController.getVelocity());

        angle = UtilMethods.clip(angle, ShotCalculator.MIN_HOOD_DEG, ShotCalculator.MAX_HOOD_DEG);
        double hoodPosition = ShotCalculator.hoodServoPosition(angle);
        hoodServo.setPosition(hoodPosition);

        TimedTelemetry.addData("Hood angle: ", angle);
        TimedTelemetry.addData("Hood servo: ", hoodPosition);
        TimedTelemetry.addData("Calculated hood: ", calculatedHoodAngle);

        PollenTurretController.setTargetVel(toggleFlywheel ? targetVel : 0);
//        PollenTurretController.setHoodAngle(hoodOffset);



        TimedTelemetry.addData("GoBilda Heading ", heading);
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


    static double wrap(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }
}
