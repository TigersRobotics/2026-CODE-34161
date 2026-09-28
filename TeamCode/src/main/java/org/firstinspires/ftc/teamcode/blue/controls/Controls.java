package org.firstinspires.ftc.teamcode.blue.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls {
    private final Gamepad driver;
    private final Gamepad operator;

    public Controls(Gamepad driver, Gamepad operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public double forward() {
        return -driver.left_stick_y;
    }

    public double strafe() {
        return -driver.left_stick_x;
    }

    public double turn() {
        return -driver.right_stick_x;
    }

    public boolean slow() {
        return driver.right_trigger > 0.5;
    }

    public boolean resetHeading() {
        return driver.optionsWasPressed();
    }

    public boolean toggleFieldCentric() {
        return driver.shareWasPressed();
    }

    public boolean autoAim() {
        return driver.right_bumper;
    }

    public boolean chasePollen() {
        return driver.left_bumper;
    }

    public boolean lockWheels() {
        return driver.x;
    }

    public boolean intakeIn() {
        return operator.right_trigger > 0.3;
    }

    public boolean intakeOut() {
        return operator.left_trigger > 0.3;
    }

    public boolean toggleLauncher() {
        return operator.aWasPressed();
    }

    public boolean shoot() {
        return operator.right_bumper;
    }

    public boolean unjam() {
        return operator.left_bumper;
    }

    public boolean toggleAutoDistance() {
        return operator.yWasPressed();
    }

    public boolean hoodUp() {
        return operator.dpadUpWasPressed();
    }

    public boolean hoodDown() {
        return operator.dpadDownWasPressed();
    }

    public boolean rpmUp() {
        return operator.dpadRightWasPressed();
    }

    public boolean rpmDown() {
        return operator.dpadLeftWasPressed();
    }

    public boolean liftFlower() {
        return operator.bWasPressed();
    }

    public boolean liftDown() {
        return operator.xWasPressed();
    }

    public double liftManual() {
        double stick = -operator.right_stick_y;
        return Math.abs(stick) > 0.1 ? stick : 0;
    }

    public void rumbleDriver(int ms) {
        driver.rumble(ms);
    }

    public void rumbleOperator(int ms) {
        operator.rumble(ms);
    }
}
