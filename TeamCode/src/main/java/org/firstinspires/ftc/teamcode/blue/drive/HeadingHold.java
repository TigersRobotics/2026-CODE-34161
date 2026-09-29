package org.firstinspires.ftc.teamcode.blue.drive;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.PID;

public class HeadingHold {
    private final PID pid = new PID(BlueConstants.HOLD_P, 0, BlueConstants.HOLD_D);
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private double target = 0;
    private boolean holding = false;
    public boolean enabled = true;

    public double update(double turnInput, double heading, boolean translating) {
        if (!enabled || Math.abs(turnInput) > BlueConstants.DEADBAND) {
            release();
            return turnInput;
        }

        if (!holding) {
            target = heading;
            if (sinceTurn.seconds() < BlueConstants.HOLD_DELAY) return 0;
            holding = true;
            pid.reset();
        }

        if (!translating) return 0;

        pid.kP = BlueConstants.HOLD_P;
        pid.kD = BlueConstants.HOLD_D;
        double error = AngleUnit.normalizeRadians(target - heading);
        return Range.clip(pid.calculate(error), -BlueConstants.HOLD_MAX, BlueConstants.HOLD_MAX);
    }

    public void release() {
        sinceTurn.reset();
        holding = false;
    }

    public boolean isHolding() {
        return holding;
    }

    public double getTarget() {
        return target;
    }
}
