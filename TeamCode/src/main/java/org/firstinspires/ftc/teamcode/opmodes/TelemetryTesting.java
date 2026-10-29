package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

@TeleOp
public class TelemetryTesting extends OpMode {

    @Override
    public void init() {
        TimedTelemetry.init(telemetry);
        Timer.init();
        TimedTelemetry.resetAll();
        TimedTelemetry.addLine("Deez NUts", 5000);
        TimedTelemetry.addLine("DN2", 1000);
    }

    @Override
    public void loop() {
        Timer.update();
        TimedTelemetry.update();
        telemetry.addData("Stuff: ", TimedTelemetry.getTelemetryList());
        telemetry.update();

    }
}
