package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.SPINDEXER_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.SPINDEXER_MAX_ROTATION;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.sensors.Colour;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class Spindexer {
    private static boolean isInitialized = false;
    public static Colour.DetectedColour[] balls = {Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY};

    /**
     * Restraints: number from 0 to 3
     */
    private static int pos = 0;
    private static double currentAngle = 0.0;
    private static Servo servo;
    private static int emptySlots = 4;
    private static ElapsedTime moveTimer = new ElapsedTime();

    public static Colour colourSensor = new Colour();

    private static final double DEG_PER_SEC = RobotSpecs.SPINDEXER_RPM * 6.0;
    private static final double TIMER_MARGIN = 0.05;
    private static final double DEGREES_PER_SLOT = 90.0;
    private static  double moveTime = 0;


    public static void init(HardwareMap hardwareMap, boolean actuallyInitialize) {
        if(actuallyInitialize) {
            servo = hardwareMap.get(Servo.class, SPINDEXER_SERVO_NAME);
            colourSensor = new Colour();
            colourSensor.init(hardwareMap);
            servo.setDirection(Servo.Direction.FORWARD);
            moveTimer = new ElapsedTime();
            isInitialized = true;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }


    /**
     * Assumes that we use a singular 360 deg servo
     * @param index the slot that you want rotated to the intake
     */
    public static void setIndex(int index) {
        if(isInitialized) {
            int newPos = Math.floorMod(index, 4);
            double baseAngle = newPos * DEGREES_PER_SLOT;
            //This is for finding the shortest path towards the position
            double bestAngle = baseAngle;
            double bestDist = Math.abs(baseAngle - currentAngle);
            for (double a = baseAngle + 360; a <= SPINDEXER_MAX_ROTATION; a += 360) {
                double distance = Math.abs(a - currentAngle);
                if (distance < bestDist) {
                    bestDist = distance;
                    bestAngle = a;
                }
            }
            if (bestDist > 0) {
                moveTime = bestDist / DEG_PER_SEC + TIMER_MARGIN; //*4 because it should be the whole rotation not just for 1 slot
                moveTimer.reset();
            }

            pos = newPos;
            currentAngle = bestAngle;
            servo.setPosition(bestAngle / SPINDEXER_MAX_ROTATION);
        } else TimedTelemetry.addLine("Attempted to set index of spindexer but its not initialized!");
    }

    /**
     * Assumes that the colour sensor is at the intake
     */
    public static void update() {
        if(isInitialized) {
            //Only read the colour sens once the slot has moved past the sensor
            if (moveTimer.seconds() > moveTime) {
                Colour.DetectedColour detectedColour = colourSensor.getDetectedColour();
                if (balls[pos] == Colour.DetectedColour.EMPTY && detectedColour != Colour.DetectedColour.EMPTY)
                    emptySlots--;
                if (balls[pos] != Colour.DetectedColour.EMPTY && detectedColour == Colour.DetectedColour.EMPTY)
                    emptySlots++;
                balls[pos] = detectedColour;
            }
        } else TimedTelemetry.addLine("Attempted to update spindexer but its not initialized! >:(");
    }

    public static boolean isFull() { return emptySlots == 0; }
    public static void clearSlot(int slot) {
        if (balls[slot] != Colour.DetectedColour.EMPTY) {
            balls[slot] = Colour.DetectedColour.EMPTY;
            emptySlots++;
        }
    }
    public static void recountEmpties() {
        emptySlots = 0;
        for(Colour.DetectedColour colour : balls) {
            if(colour == Colour.DetectedColour.EMPTY) emptySlots ++;
        }
    }

    public static void stop() {
        isInitialized = false;
        balls = new Colour.DetectedColour[]{Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY, Colour.DetectedColour.EMPTY};
        servo = null;
        moveTimer = null;
        colourSensor = null;
    }

}
