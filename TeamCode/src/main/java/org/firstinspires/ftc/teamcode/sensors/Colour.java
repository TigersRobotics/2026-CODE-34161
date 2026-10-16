package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;

public class Colour {

    NormalizedColorSensor colourSensor;

    public enum DetectedColour{
        RED_NECTAR,
        BLUE_NECTAR,
        POLLEN,
        EMPTY
    }

    public void init(HardwareMap hwMap){
        colourSensor = hwMap.get(NormalizedColorSensor.class, HardwareNames.COLOUR_SENSOR);

        //Adjust number till number is not too high and not too low
        colourSensor.setGain(1);
    }

    public DetectedColour getDetectedColour(Telemetry telemetry){
        NormalizedRGBA colours = colourSensor.getNormalizedColors(); //Returns 4 values: red, green, blue, alpha(Brightness)

        float normRed, normGreen, normBlue;
        normRed = colours.red / colours.alpha;
        normGreen = colours.green / colours.alpha;
        normBlue = colours.blue / colours.alpha;

        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);

        //add if statements for any other colour added
        /*
        Values of colours, ex. >.35, <.3, <.3
        red, blue, green
        RED_NECTAR =
        BLUE_NECTAR =
        POLLEN =
         */

        /* FINISH THIS ONCE YOU GET ALL THE VALUES
        if (normRed > Insert Value && normBlue < Insert Value && normGreen < Insert Value){
            return DetectedColour.RED_NECTAR;
        }
        else if (normRed > Insert Value && normBlue < Insert Value && normGreen < Insert Value){
            return DetectedColour.BLUE_NECTAR;
        }
        else if (normRed > Insert Value && normBlue < Insert Value && normGreen < Insert Value){
            return DetectedColour.POLLEN;
        }
        else{
            return DetectedColour.EMPTY;
        }
         */

        return DetectedColour.EMPTY;
    }
}
