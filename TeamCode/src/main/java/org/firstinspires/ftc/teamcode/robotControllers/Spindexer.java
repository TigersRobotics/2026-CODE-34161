package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.SPINDEXER_SERVO_NAME;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.Colour;

public class Spindexer {
    public Colour.DetectedColour[] balls = {Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY};
    private Telemetry telemetry;

    /**
     * Restraints: number from 0 to 3
     */
    public int pos = 0;

    private final Servo servo;


    public Colour colourSensor = new Colour();



    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, SPINDEXER_SERVO_NAME);
        colourSensor.init(hardwareMap);
    }




    /**
     * Assumes that we use a singular 360 deg servo
     * @param index the index that the ball is at
     */
    public void setIndex(int index) {
        this.pos = index%4;
        servo.setPosition(((double) pos) /4);
    }

    /**
     * Assumes that the colour sensor is at the intake
     */
    public void update() {
        int estimatedPos = Math.toIntExact((long) Math.floor(servo.getPosition() * 8));
        int viewIndex = (estimatedPos/2);
        ///If the number is odd it would be in the transition between two indexes, so dont update if its odd

        if(viewIndex % 2 == 1) balls[viewIndex] = colourSensor.getDetectedColour(telemetry);

        for (Colour.DetectedColour colour: balls) {
            String status = "";
            switch (colour) {
                case EMPTY: status = "EMPTY"; break;
                case POLLEN: status = "POLLEN"; break;
                case RED_NECTAR: status = "RED_NECTAR"; break;
                case BLUE_NECTAR: status = "BLUE_NECTAR"; break;
            }

            telemetry.addLine("Ball " + status);
        }
    }


}
