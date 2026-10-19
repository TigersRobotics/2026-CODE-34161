package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.SPINDEXER_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.SPINDEXER_MAX_ROTATION;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.sensors.Colour;

public class Spindexer {
    public Colour.DetectedColour[] balls = {Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY};
    private Telemetry telemetry;

    /**
     * Restraints: number from 0 to 3
     */
    private int pos = 0;
    private double currentAngle = 0.0;
    private final Servo servo;
    private int emptySlots = 4;
    private ElapsedTime moveTimer = new ElapsedTime();

    public Colour colourSensor = new Colour();

    private static final double DEG_PER_SEC = RobotSpecs.SPINDEXER_RPM * 6.0;
    private static final double TIMER_MARGIN = 0.05;
    private static final double DEGREES_PER_SLOT = 90.0;
    private double moveTime = 0;


    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, SPINDEXER_SERVO_NAME);
        colourSensor.init(hardwareMap);
        servo.setDirection(Servo.Direction.FORWARD);
        this.telemetry = telemetry;
        moveTimer.reset();
    }




    /**
     * Assumes that we use a singular 360 deg servo
     * @param index the slot that you want rotated to the intake
     */
    public void setIndex(int index) {
        int newPos = Math.floorMod(index, 4);
        double baseAngle = newPos * DEGREES_PER_SLOT;
        //This is for finding the shortest path towards the position
        double bestAngle = baseAngle;
        double bestDist = Math.abs(baseAngle - currentAngle);
        for(double a = baseAngle + 360; a<=SPINDEXER_MAX_ROTATION; a += 360) {
            double distance = Math.abs(a - currentAngle);
            if(distance < bestDist) {
                bestDist = distance;
                bestAngle = a;
            }
        }
        if(bestDist > 0) {
            moveTime = bestDist / DEG_PER_SEC + TIMER_MARGIN; //*4 because it should be the whole rotation not just for 1 slot
            moveTimer.reset();
        }

        pos = newPos;
        currentAngle=bestAngle;
        servo.setPosition(bestAngle/ SPINDEXER_MAX_ROTATION);
    }

    /**
     * Assumes that the colour sensor is at the intake
     */
    public void update() {
        //Only read the colour sens once the slot has moved past the sensor
        if(moveTimer.seconds() > moveTime) {
            Colour.DetectedColour detectedColour = colourSensor.getDetectedColour(telemetry);
            if(balls[pos] == Colour.DetectedColour.EMPTY && detectedColour != Colour.DetectedColour.EMPTY) emptySlots --;
            if(balls[pos] != Colour.DetectedColour.EMPTY && detectedColour == Colour.DetectedColour.EMPTY) emptySlots++;
            balls[pos] = detectedColour;
        }
    }

    public boolean isFull() { return emptySlots == 0; }
    public void clearSlot(int slot) {
        if (balls[slot] != Colour.DetectedColour.EMPTY) {
            balls[slot] = Colour.DetectedColour.EMPTY;
            emptySlots++;
        }
    }
    public void recountEmpties() {
        emptySlots = 0;
        for(Colour.DetectedColour colour : balls) {
            if(colour == Colour.DetectedColour.EMPTY) emptySlots ++;
        }
    }

}
