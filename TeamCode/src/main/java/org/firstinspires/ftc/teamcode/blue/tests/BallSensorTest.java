package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blue.mechanisms.BallSensor;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

@TeleOp(name = "Ball Sensor Test", group = "Blue Test")
public class BallSensorTest extends OpMode {
    BallSensor sensor;
    DataLogger logger;

    String[] labels = {"pollen", "red_nectar", "blue_nectar", "empty"};
    int label = 0;
    int samples = 0;
    int correct = 0;

    @Override
    public void init() {
        sensor = new BallSensor(hardwareMap);
        logger = new DataLogger("ball_sensor", "label", "hue", "sat", "val", "distanceCm", "guess");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) label = (label + 1) % labels.length;
        if (gamepad1.dpadLeftWasPressed()) label = (label + labels.length - 1) % labels.length;

        float[] hsv = sensor.getHsv();
        double distance = sensor.getDistance();
        Target.Type guess = sensor.getBall();
        String guessName = guess == null ? "empty" : guess.toString().toLowerCase();

        if (gamepad1.aWasPressed()) {
            logger.log(labels[label], String.format("%.1f", hsv[0]), String.format("%.3f", hsv[1]),
                    String.format("%.3f", hsv[2]), String.format("%.2f", distance), guessName);
            samples++;
            if (guessName.equals(labels[label])) correct++;
        }

        telemetry.addLine("Hold a ball in front, dpad L/R = what it is, A = save sample");
        telemetry.addData("Label", labels[label]);
        telemetry.addData("Hue", "%.1f", hsv[0]);
        telemetry.addData("Sat", "%.3f", hsv[1]);
        telemetry.addData("Val", "%.3f", hsv[2]);
        telemetry.addData("Distance cm", "%.2f", distance);
        telemetry.addData("Guess", guessName);
        telemetry.addData("Samples", samples);
        telemetry.addData("Guessed right", correct + " / " + samples);
        telemetry.addData("Log", logger.getPath());
    }

    @Override
    public void stop() {
        logger.close();
    }
}
