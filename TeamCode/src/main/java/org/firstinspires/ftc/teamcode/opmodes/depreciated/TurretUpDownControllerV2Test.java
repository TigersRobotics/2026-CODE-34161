package org.firstinspires.ftc.teamcode.opmodes.depreciated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.TurretUpAngleControllerV2;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

@Disabled
@Deprecated
@TeleOp (name = "Servo Test 2")
public class TurretUpDownControllerV2Test extends OpMode {
    @Override
    public void init() {
        TimedTelemetry.init(telemetry);
        Timer.init();
        TurretUpAngleControllerV2.init(hardwareMap);
    }

    @Override
    public void loop() {
        Timer.update();
        TimedTelemetry.update();
        TurretUpAngleControllerV2.update(gamepad1.left_stick_y);
        telemetry.update();
    }
}
