package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;

public class Colour {

    NormalizedColorSensor colourSensor;
    float gain = 0;
    public enum DetectedColour{
        RED_NECTAR,
        BLUE_NECTAR,
        POLLEN,
        EMPTY
    }

    public void init(HardwareMap hwMap){
        colourSensor = hwMap.get(NormalizedColorSensor.class, HardwareNames.COLOUR_SENSOR);

        //Adjust number till number is not too high and not too low
        colourSensor.setGain(8);
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
        red, green, blue
        RED_NECTAR = >0.11, <0.085, <0.045
        BLUE_NECTAR = <0.025, <0.05, >0.11
        POLLEN = >0.11, >1.6, <0.07
         */


        if (normRed > 0.11 && normGreen < 0.085 && normBlue < 0.045){
            return DetectedColour.RED_NECTAR;
        }
        else if (normRed < 0.025 && normGreen < 0.05 && normBlue > 0.11){
            return DetectedColour.BLUE_NECTAR;
        }
        else if (normRed > 0.11 && normGreen > 0.16 && normBlue < 0.07){
            return DetectedColour.POLLEN;
        }
        else{
            return DetectedColour.EMPTY;
        }

    }
}
