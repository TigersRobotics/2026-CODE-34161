package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER);
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeController.init(hardwareMap, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, ENABLE_INTAKE_CONTROLLER);

        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, TuningConstants.FLYWHEEL_F_VALUE);
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        double intakePower = enableIntake ? 1 : 0;
        Timer.update();
        if(gamepad1.circleWasPressed()) toggleFlywheel = !toggleFlywheel;
        if(gamepad1.dpadUpWasPressed()) targetVel += 10;
        if(gamepad1.dpadDownWasPressed()) targetVel -= 10;


        TimedTelemetry.addData("Target Velocity: ", targetVel);

        PollenTurretController.setTargetVel(toggleFlywheel ? targetVel : 0);

        if(gamepad1.dpad_right) yaw += Timer.DTime()*0.1;
        if(gamepad1.dpad_left) yaw -= Timer.DTime()*0.1;
        PollenTurretController.setYaw(yaw);

        TimedTelemetry.addData("Target Yaw: ", yaw);

        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0));
        PollenTurretController.update();
        UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);
        SingleIntakeController.update(intakePower);
        TimedTelemetry.update();

        telemetry.update();
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeController.stop();
        SingleIntakeController.stop();
    }
}
