package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeServoController;
import org.firstinspires.ftc.teamcode.sensors.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@TeleOp
public class TeleOpMain extends OpMode {
    final boolean ENABLE_TURRET_CONTROLLER = true;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;
    final boolean ENABLE_WEBCAM = false;
    boolean enableIntake = false;
    double yaw = 0;
    boolean toggleFlywheel = false;
    double targetVel = 1200;

    GoBildaOdometry odometry;
    int hiveIndex = 1;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER);
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
        odometry.update();

        double posx = odometry.getX(DistanceUnit.INCH);
        double posY = odometry.getY(DistanceUnit.INCH);
        double heading = odometry.getHeading(AngleUnit.RADIANS);

        double calculatedYaw = calculateTurretAngle(hiveIndex, posx, posY, heading);
        double calculatedYawDeg = Math.toDegrees(calculatedYaw);


        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        double intakePower = enableIntake ? 1 : 0;
        Timer.update();
        if(gamepad1.circleWasPressed()) toggleFlywheel = !toggleFlywheel;
        if(gamepad1.dpadUpWasPressed()) targetVel += 10;
        if(gamepad1.dpadDownWasPressed()) targetVel -= 10;
        if(gamepad1.triangleWasPressed()) hiveIndex = (hiveIndex + 1) % 2;


        TimedTelemetry.addData("Target Velocity: ", targetVel);

        PollenTurretController.setTargetVel(toggleFlywheel ? targetVel : 0);

        if(gamepad1.dpad_right) yaw += Timer.DTime()*0.1;
        if(gamepad1.dpad_left) yaw -= Timer.DTime()*0.1;
        double realYaw = calculatedYawDeg - yaw;

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


        while(calculatedYaw > 1.5*Math.PI) calculatedYaw -= 2 * Math.PI;
        while(calculatedYaw < -1.5*Math.PI) calculatedYaw += 2* Math.PI;

        return calculatedYaw;
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeController.stop();
        SingleIntakeController.stop();
    }
}
