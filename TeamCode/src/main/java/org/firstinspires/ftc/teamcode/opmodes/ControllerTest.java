package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.TurretControllerWithHood;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeServoController;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

@TeleOp
public class ControllerTest extends OpMode {

    final boolean ENABLE_TURRET_CONTROLLER = false;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_SERVO_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;

    @Override
    public void init() {
        TimedTelemetry.init(telemetry);

        MecanumDriveTrainController.init(hardwareMap, ENABLE_DRIVE_CONTROLLER);
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeServoController.init(hardwareMap, ENABLE_UPTAKE_SERVO_CONTROLLER);
        SingleIntakeController.init(hardwareMap, ENABLE_INTAKE_CONTROLLER);

    }
    @Override
    public void loop() {

        MecanumDriveTrainController.update(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        PollenTurretController.update();
        UptakeServoController.update();
        SingleIntakeController.update(gamepad1.left_trigger);

        TimedTelemetry.update();
        telemetry.update();
    }

}
