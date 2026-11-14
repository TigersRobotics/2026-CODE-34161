package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeServoController;
import org.firstinspires.ftc.teamcode.sensors.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@TeleOp
public class TeleOpMain extends OpMode {
    final boolean ENABLE_TURRET_CONTROLLER = false;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = false;
    final boolean ENABLE_INTAKE_CONTROLLER = true;
    final boolean ENABLE_WEBCAM = false;
    boolean enableIntake = false;

    @Override
    public void init() {

        MecanumDriveTrainController.init(hardwareMap, telemetry, ENABLE_DRIVE_CONTROLLER);
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeServoController.init(hardwareMap, telemetry, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, telemetry, ENABLE_INTAKE_CONTROLLER);
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()) enableIntake = !enableIntake;
        double intakePower = enableIntake ? 1 : 0;

        MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y,2), UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x,2.0));
        PollenTurretController.update();
        UptakeServoController.update();
        SingleIntakeController.update(intakePower);

    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeServoController.stop();
        SingleIntakeController.stop();
    }
}
