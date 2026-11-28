package org.firstinspires.ftc.teamcode.opmodes.depreciated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

@Disabled
@Deprecated
@TeleOp (name = "Driving Test")
public class DriveTest extends OpMode {
    @Override
    public void init() {

        Timer.init();
        TimedTelemetry.init(telemetry);
        MecanumDriveTrainController.init(hardwareMap, true);
    }

    @Override
    public void loop() {
        Timer.update();
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        MecanumDriveTrainController.update(y,x, turn);

        TimedTelemetry.update();
        telemetry.update();

    }
}
