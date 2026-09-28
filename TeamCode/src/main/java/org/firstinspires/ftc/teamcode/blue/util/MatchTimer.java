package org.firstinspires.ftc.teamcode.blue.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MatchTimer {
    private final ElapsedTime timer = new ElapsedTime();
    private boolean flowerWarned = false;
    private boolean endWarned = false;

    public void start() {
        timer.reset();
        flowerWarned = false;
        endWarned = false;
    }

    public double secondsLeft() {
        return Math.max(0, BlueConstants.TELEOP_LENGTH - timer.seconds());
    }

    public boolean flowersOpen() {
        return secondsLeft() <= 60;
    }

    public void update(Gamepad driver, Gamepad operator) {
        if (!flowerWarned && secondsLeft() <= 62) {
            flowerWarned = true;
            driver.rumbleBlips(3);
            operator.rumbleBlips(3);
        }
        if (!endWarned && secondsLeft() <= 20) {
            endWarned = true;
            driver.rumble(1000);
            operator.rumble(1000);
        }
    }

    public String display() {
        int left = (int) Math.ceil(secondsLeft());
        return String.format("%d:%02d%s", left / 60, left % 60, flowersOpen() ? "  FLOWERS OPEN" : "");
    }
}
