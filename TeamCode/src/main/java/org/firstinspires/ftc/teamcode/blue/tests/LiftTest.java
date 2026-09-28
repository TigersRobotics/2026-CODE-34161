package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blue.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

@TeleOp(name = "Lift Test", group = "Blue Test")
public class LiftTest extends OpMode {
    Lift lift;
    boolean manual = false;
    int savedFlower = -1;
    int savedMax = -1;
    int normalMax;
    int normalDown;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
        normalMax = BlueConstants.LIFT_MAX;
        normalDown = BlueConstants.LIFT_DOWN;
    }

    @Override
    public void loop() {
        double stick = -gamepad1.left_stick_y;
        boolean ignoreLimits = gamepad1.left_bumper;

        if (ignoreLimits) {
            BlueConstants.LIFT_MAX = Integer.MAX_VALUE;
            BlueConstants.LIFT_DOWN = Integer.MIN_VALUE;
        } else {
            BlueConstants.LIFT_MAX = normalMax;
            BlueConstants.LIFT_DOWN = normalDown;
        }

        if (Math.abs(stick) > 0.1) {
            lift.manual(stick * 0.6);
            manual = true;
        } else if (manual) {
            lift.holdHere();
            manual = false;
        }

        if (gamepad1.aWasPressed()) lift.down();
        if (gamepad1.yWasPressed()) lift.flower();
        if (gamepad1.xWasPressed()) savedFlower = lift.getPosition();
        if (gamepad1.bWasPressed()) savedMax = lift.getPosition();
        if (gamepad1.backWasPressed()) lift.resetEncoder();

        telemetry.addLine("Left stick = move, hold LB = ignore limits");
        telemetry.addLine("A = down, Y = flower, X = save flower, B = save max, BACK = zero");
        telemetry.addData("Position", lift.getPosition());
        telemetry.addData("Target", lift.getTarget());
        telemetry.addData("Busy", lift.isBusy());
        telemetry.addLine();
        telemetry.addLine("Copy into BlueConstants:");
        telemetry.addData("LIFT_FLOWER", savedFlower);
        telemetry.addData("LIFT_MAX", savedMax);
    }

    @Override
    public void stop() {
        BlueConstants.LIFT_MAX = normalMax;
        BlueConstants.LIFT_DOWN = normalDown;
        lift.stop();
    }
}
