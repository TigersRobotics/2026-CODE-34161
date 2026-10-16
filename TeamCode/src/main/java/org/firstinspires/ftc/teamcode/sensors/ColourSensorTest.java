package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ColourSensorTest extends OpMode {

    Colour test = new Colour();

    Colour.DetectedColour detectedColour;

    @Override
    public void init() {
        test.init(hardwareMap);
    }

    @Override
    public void loop() {
        test.getDetectedColour(telemetry);
        telemetry.addData("Colour Detected:", detectedColour);
    }
}
