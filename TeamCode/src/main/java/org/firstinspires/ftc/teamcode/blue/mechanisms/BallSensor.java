package org.firstinspires.ftc.teamcode.blue.mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

public class BallSensor {
    private final NormalizedColorSensor sensor;
    private final float[] hsv = new float[3];

    public BallSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "ballColor");
        sensor.setGain(BlueConstants.COLOR_GAIN);
    }

    public float[] getHsv() {
        Color.colorToHSV(sensor.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    public double getDistance() {
        if (sensor instanceof DistanceSensor) {
            return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        }
        return -1;
    }

    public boolean hasBall() {
        double d = getDistance();
        return d >= 0 && d < BlueConstants.BALL_DISTANCE;
    }

    public Target.Type getBall() {
        if (!hasBall()) return null;

        float hue = getHsv()[0];
        if (hue >= BlueConstants.YELLOW_MIN && hue <= BlueConstants.YELLOW_MAX) return Target.Type.POLLEN;
        if (hue >= BlueConstants.BLUE_MIN && hue <= BlueConstants.BLUE_MAX) return Target.Type.BLUE_NECTAR;
        if (hue <= BlueConstants.RED_LOW || hue >= BlueConstants.RED_HIGH) return Target.Type.RED_NECTAR;
        return Target.Type.UNKNOWN;
    }
}
