package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blue.controls.Controls;

@TeleOp(name = "Controls Test", group = "Blue Test")
public class ControlsTest extends OpMode {
    Controls controls;
    int launcherPresses = 0;
    int heading = 0;
    int fieldCentric = 0;
    int autoDistance = 0;
    int hood = 0;
    int rpm = 0;
    int liftFlower = 0;
    int liftDown = 0;

    @Override
    public void init() {
        controls = new Controls(gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        if (controls.resetHeading()) heading++;
        if (controls.toggleFieldCentric()) fieldCentric++;
        if (controls.toggleLauncher()) launcherPresses++;
        if (controls.toggleAutoDistance()) autoDistance++;
        if (controls.hoodUp()) hood++;
        if (controls.hoodDown()) hood--;
        if (controls.rpmUp()) rpm += 100;
        if (controls.rpmDown()) rpm -= 100;
        if (controls.liftFlower()) liftFlower++;
        if (controls.liftDown()) liftDown++;

        telemetry.addLine("DRIVER (gamepad 1)");
        telemetry.addData("forward", "%.2f", controls.forward());
        telemetry.addData("strafe", "%.2f", controls.strafe());
        telemetry.addData("turn", "%.2f", controls.turn());
        telemetry.addData("slow (RT)", controls.slow());
        telemetry.addData("auto aim (RB)", controls.autoAim());
        telemetry.addData("chase pollen (LB)", controls.chasePollen());
        telemetry.addData("lock wheels (X)", controls.lockWheels());
        telemetry.addData("reset heading (options)", heading);
        telemetry.addData("field centric (share)", fieldCentric);

        telemetry.addLine();
        telemetry.addLine("OPERATOR (gamepad 2)");
        telemetry.addData("intake in (RT)", controls.intakeIn());
        telemetry.addData("intake out (LT)", controls.intakeOut());
        telemetry.addData("shoot (RB)", controls.shoot());
        telemetry.addData("unjam (LB)", controls.unjam());
        telemetry.addData("launcher toggle (A)", launcherPresses);
        telemetry.addData("auto distance (Y)", autoDistance);
        telemetry.addData("hood (dpad U/D)", hood);
        telemetry.addData("rpm (dpad L/R)", rpm);
        telemetry.addData("lift flower (B)", liftFlower);
        telemetry.addData("lift down (X)", liftDown);
        telemetry.addData("lift manual (right stick)", "%.2f", controls.liftManual());

        if (gamepad1.guideWasPressed()) controls.rumbleDriver(300);
        if (gamepad2.guideWasPressed()) controls.rumbleOperator(300);
    }
}
