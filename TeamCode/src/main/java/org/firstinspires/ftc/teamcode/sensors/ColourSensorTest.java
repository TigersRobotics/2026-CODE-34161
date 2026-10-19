package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class ColourSensorTest extends OpMode {

    Colour test = new Colour();

    Colour.DetectedColour detectedColour;

    @Override
    public void init() {
        test.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColour = test.getDetectedColour(telemetry);

        String name = "Empty";

        switch (detectedColour){
            case RED_NECTAR: name = "red"; break;
            case BLUE_NECTAR: name = "blue"; break;
            case POLLEN: name = "pollen"; break;
        }

        telemetry.addLine("Colour Detected: " + name);
    }
}
