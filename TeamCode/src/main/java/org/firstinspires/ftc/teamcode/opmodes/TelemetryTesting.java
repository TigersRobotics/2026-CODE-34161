package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        telemetry.addData("Stuff: ", TimedTelemetry.getTelemetryListAsStrings());
        TimedTelemetry.addData("Caption", "%.2f", 1, 10.1231 , 10.932);
        TimedTelemetry.update();
        Timer.update();
        telemetry.update();

    }
    public int test(int ao) {
        return ao^2;
    }

    public void test(){{
        // this is a test to show another team about merge conflict
    }}
}
