package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_UP_ANGLE_SERVO_NAME;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
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
    private static final double HOOD_SERVO_RANGE_DEG = 20;
    private boolean tipped;
    final boolean ENABLE_TURRET_CONTROLLER = true;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;

    boolean enableIntake = false;
    double yaw = 0;
    boolean toggleFlywheel = false;
    double targetVel = 1320;

    double targetVelOffset = 0;
    FieldOdometry odometry;
    int hiveIndex = 1;
    List<LynxModule> hubs;


    double posx;
    double posY;
    double heading = 90;
    double intakePower;
    double angle = 15;
    boolean reverseIntake = false;

    double P = TuningConstants.FLYWHEEL_P_VALUE, I = 0  , D = TuningConstants.FLYWHEEL_D_VALUE, F = TuningConstants.FLYWHEEL_F_VALUE;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        // Each hub now answers every encoder/velocity read in a loop from one bulk read,
        // instead of one hub round trip per read. The cache is cleared at the top of loop().
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER, false);
        ShotCalculator.loadTuning(new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt"));
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeController.init(hardwareMap, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, ENABLE_INTAKE_CONTROLLER);

        hoodServo = hardwareMap.get(Servo.class, TURRET_UP_ANGLE_SERVO_NAME);
        odometry = new FieldOdometry(new GoBildaOdometry(hardwareMap, ODOMETRY_NAME));
        odometry.zero(-72 + 6.5, -72 + 9, 90);

        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, 0, D, TuningConstants.FLYWHEEL_F_VALUE);

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




        // Wrapping the offset stops it winding up past a full turn, and wrapping the final angle
        // lets the turret carry on round instead of sticking at +/-180


        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0), Math.toDegrees(odometry.getHeading()));
        PollenTurretController.update();
        UptakeController.update(gamepad2.right_trigger - gamepad2.left_trigger);
        SingleIntakeController.update(intakePower);
        TimedTelemetry.update();

        telemetry.update();
    }

    private void handleDriverInputs() {
        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        if(gamepad1.rightBumperWasPressed()) reverseIntake = !reverseIntake;
        intakePower = enableIntake ? reverseIntake ? -1 : 1 : 0;
    }

    private void handleTechnicianInputs() {
        if(gamepad2.rightBumperWasPressed()) targetVelOffset += 5;
        if(gamepad2.leftBumperWasPressed()) targetVelOffset -= 5;
        if(gamepad2.triangleWasPressed()) hiveIndex = (hiveIndex + 1) % 2;
        if(gamepad2.circleWasPressed()) toggleFlywheel = !toggleFlywheel;

        if(gamepad2.dpadRightWasPressed()) {
            I += 0.01;
            PollenTurretController.setPFCoefficients(P, I, D, F);
        }
        if(gamepad2.dpadLeftWasPressed()) {
            I -= 0.01;
            PollenTurretController.setPFCoefficients(P, I, D, F);
        }

        // Manual turret offset. A tap is a fixed small step, and it uses the buffered button press
        // so a tap shorter than one loop is not lost. Holding the button sweeps after a short delay.
        yaw += UtilMethods.expo(gamepad2.right_stick_x , 3) * Timer.DTime() * 0.25;

        TimedTelemetry.addData("I Value: ", I);

    }

    private void handleTurret() {

        double[] t = Field.upCell(true, tipped);
        double fieldDeg = Math.toDegrees(Math.atan2(t[1] - odometry.getY(), t[0] - odometry.getX())); // deg
        double turret = wrap(Math.toDegrees(odometry.getHeading()) - fieldDeg + yaw); // deg
        if (odometry.isZeroed()) PollenTurretController.setYaw(turret);

        double closestSpeed = targetVel;
        double val = Double.MAX_VALUE;

        for (int[] pos :TuningConstants.positions) {
            double flywheelSpeed = pos[0];
            double x = pos[1] - Math.abs(odometry.getX());
            double y = pos[2] - Math.abs(odometry.getY());

            double dist = 0;
            dist += x*x + y*y;
            if(dist < val) {
                val = dist;
                closestSpeed = flywheelSpeed;
            }


        }
        closestSpeed += targetVelOffset;

        setHoodAngle(20);

        PollenTurretController.setTargetVel(toggleFlywheel ? closestSpeed + targetVelOffset : 0);
        TimedTelemetry.addData("Target Velocity: ", toggleFlywheel ? closestSpeed + targetVelOffset : 0);
        TimedTelemetry.addData("Error: ", closestSpeed - PollenTurretController.getVelocity());
    }


    @Override
    public void stop() {
        PollenTurretController.setYaw(0);
        MecanumDriveTrainController.stop();

        PollenTurretController.stop();
        UptakeController.stop();
        SingleIntakeController.stop();
    }


    static double wrap(double deg) {
        deg %= 360;
        while (deg > 160) deg -= 360;
        while (deg < -190) deg += 360;
        return deg;
    }

    public void setHoodAngle(double degrees) {
        if (hoodServo == null || Double.isNaN(degrees)) return;

        double position = UtilMethods.clip((degrees - TurretConstants.HOOD_MIN_DEG) / HOOD_SERVO_RANGE_DEG, 0, 1);
        hoodServo.setPosition(position);
    }

}
