package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.TurretUpAngleController;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;


@TeleOp (name = "Turret Up/Down Controller Test")
public class TurretUpDownControllerV1Test extends OpMode {
    @Override
    public void init() {
        TimedTelemetry.init(telemetry);
        Timer.init();
        TurretUpAngleController.init(hardwareMap);
    }

    @Override
    public void loop() {
        Timer.update();
        TimedTelemetry.update();
        TurretUpAngleController.update(gamepad1.left_stick_y);
        telemetry.update();
    }

}
