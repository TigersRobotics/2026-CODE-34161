package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

@TeleOp
public class TelemetryTesting extends OpMode {

    @Override
    public void init() {
        TimedTelemetry.init(telemetry);

        TimedTelemetry.addLine("Deez NUts", 5000);
    }

    @Override
    public void loop() {
        TimedTelemetry.update();
        telemetry.addData("Stuff: ", TimedTelemetry.getTelemetryList());
        telemetry.update();

    }
}
