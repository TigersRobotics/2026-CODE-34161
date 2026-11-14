package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Flywheel PIDF Tuning")
public class FlywheelPIDFTuning extends OpMode {

    // Change this to match your hardware config name
    private static final String MOTOR_NAME = "flywheel";

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    private double currentTargetVelocity = highVelocity;

    private DcMotorEx flywheel;

    private double P = 0;
    private double F = 0;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // flywheel.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if needed

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F));

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // Toggle target velocity
        if (gamepad1.circleWasPressed()) {
            currentTargetVelocity = (currentTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // Cycle step size
        if (gamepad1.triangleWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.squareWasPressed()) stepIndex = (stepIndex -1 + stepSizes.length) % stepSizes.length;

        // F tuning
        if (gamepad1.leftTriggerWasPressed()) F += stepSizes[stepIndex];
        if (gamepad1.rightTriggerWasPressed()) F -= stepSizes[stepIndex];

        // P tuning
        if (gamepad1.leftBumperWasPressed()) P += stepSizes[stepIndex];
        if (gamepad1.rightBumperWasPressed()) P -= stepSizes[stepIndex];

        // Apply coefficients and target velocity
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F));
        flywheel.setVelocity(currentTargetVelocity);

        double currVel = flywheel.getVelocity();
        double err = currentTargetVelocity - currVel;

        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currVel);
        telemetry.addData("Error", "%.2f", err);
        telemetry.addLine("-----------------------");
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f (index %d)", stepSizes[stepIndex], stepIndex);
        telemetry.addLine("-----------------------");
        telemetry.addLine("Circle: toggle target | X: step size");
        telemetry.addLine("LT/RT: F +/- | LB/RB: P +/-");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.setVelocity(0);
    }
}