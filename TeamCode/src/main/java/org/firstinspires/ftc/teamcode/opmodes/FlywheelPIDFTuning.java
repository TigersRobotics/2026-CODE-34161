package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;

@TeleOp(name = "Flywheel PIDF Tuning")
public class FlywheelPIDFTuning extends OpMode {

    // Change this to match your hardware config name
    private static final String MOTOR_NAME = "flywheel";

    public double highVelocity = 1500;
    private double currentTargetVelocity = highVelocity;

    private DcMotorEx flywheel;

    private double P = 11.0;
    private double F = 13.698;

    // What the hub was last told. NaN means the target velocity has not been sent yet.
    private double sentP = P;
    private double sentF = F;
    private double sentTargetVelocity = Double.NaN;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 1;

    private Servo hoodServo;
    private double servoPower = 1;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//         flywheel.setDirection(DcMotorSimple.Direction.REVERSE); // uncomment if needed

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F));

        hoodServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        hoodServo.setPosition(servoPower);
        //Change target Velocity
        if(gamepad1.circleWasPressed()) currentTargetVelocity += 10;
        if(gamepad1.crossWasPressed()) currentTargetVelocity -= 10;

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

        // Apply coefficients and target velocity, but only when a button changed one of them.
        // Sending them every loop slowed the loop down, which makes the velocity readout laggier.
        if (P != sentP || F != sentF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, 0, 0, F));
            sentP = P;
            sentF = F;
        }
        if (currentTargetVelocity != sentTargetVelocity) {
            flywheel.setVelocity(currentTargetVelocity);
            sentTargetVelocity = currentTargetVelocity;
        }

        if(gamepad1.left_stick_y > 0.75) servoPower += 0.001;
        if(gamepad1.left_stick_y < -0.75) servoPower -= 0.001;

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
