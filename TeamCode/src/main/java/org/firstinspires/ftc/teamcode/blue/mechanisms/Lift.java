package org.firstinspires.ftc.teamcode.blue.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

public class Lift {
    private final DcMotorEx motor;
    private int target = 0;

    public Lift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goTo(int ticks) {
        target = Math.max(BlueConstants.LIFT_DOWN, Math.min(BlueConstants.LIFT_MAX, ticks));
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(BlueConstants.LIFT_POWER);
    }

    public void down() {
        goTo(BlueConstants.LIFT_DOWN);
    }

    public void flower() {
        goTo(BlueConstants.LIFT_FLOWER);
    }

    public void manual(double power) {
        int pos = getPosition();
        if ((power > 0 && pos >= BlueConstants.LIFT_MAX) || (power < 0 && pos <= BlueConstants.LIFT_DOWN)) {
            power = 0;
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
        target = pos;
    }

    public void holdHere() {
        goTo(getPosition());
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        target = 0;
    }

    public void stop() {
        motor.setPower(0);
    }
}
