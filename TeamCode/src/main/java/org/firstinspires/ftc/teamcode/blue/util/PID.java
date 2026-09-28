package org.firstinspires.ftc.teamcode.blue.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double kP, kI, kD;

    private double integral = 0;
    private double lastError = 0;
    private boolean first = true;
    private final ElapsedTime timer = new ElapsedTime();

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double error) {
        double dt = timer.seconds();
        timer.reset();

        if (first || dt <= 0 || dt > 0.5) {
            first = false;
            lastError = error;
            return kP * error;
        }

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
    }
}
