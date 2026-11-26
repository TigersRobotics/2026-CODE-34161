package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

@TeleOp(name = "Test")
public class Test extends OpMode {

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);
        TimedTelemetry.addLine("<small>1234567890123456789012345678901234567890123456789012345678901234567890</small>");
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void loop() {
//        if (gamepad1.circle) {
//            TimedTelemetry.addLine(Ascii.A);
//        } else if (gamepad1.cross) {
//            TimedTelemetry.addLine(Ascii.B);
//        } else if (gamepad1.triangle) {
//            TimedTelemetry.addLine(Ascii.C);
//        } else if (gamepad1.square) {
//
//            TimedTelemetry.addLine(Ascii.Jack);
//        }
        Timer.update();
        TimedTelemetry.update();
        telemetry.update();
    }
}
