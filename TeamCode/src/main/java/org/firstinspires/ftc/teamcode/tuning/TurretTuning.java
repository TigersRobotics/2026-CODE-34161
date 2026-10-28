package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotControllers.TurretControllerWithHood;

@TeleOp
@Disabled ///Outdated using the old thing not for current robot
@Deprecated
public class TurretTuning extends OpMode {
    TurretControllerWithHood turretController;

    public DcMotorEx flywheelMotorA;
    public DcMotorEx flywheelMotorB;
    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double currentTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    public Servo flywheelCompressionServoA;
    public Servo flywheelCompressionServoB;
    double flywheelCompressionPower = 0;


    @Override
    public void init() {
        flywheelMotorA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorA.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotorB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        flywheelMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorB.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelCompressionServoA = hardwareMap.get(Servo.class, "flywheelCompressionA");
        flywheelCompressionServoB = hardwareMap.get(Servo.class, "flywheelCompressionB");
        flywheelCompressionServoA.setDirection(Servo.Direction.REVERSE);
        flywheelCompressionServoB.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Turret Tuning Initialized");
    }

    @Override
    public void loop() {
        if(currentTargetVelocity == highVelocity) {
            currentTargetVelocity = lowVelocity;
        } else { currentTargetVelocity = highVelocity; }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.leftTriggerWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.rightTriggerWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.leftBumperWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if(gamepad1.rightBumperWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()) {
            flywheelCompressionPower -= stepSizes[stepIndex];
            flywheelCompressionPower = Math.max(0, flywheelCompressionPower);
        }
        if(gamepad1.dpadRightWasPressed()) {
            flywheelCompressionPower += stepSizes[stepIndex];
            flywheelCompressionPower = Math.min(1, flywheelCompressionPower);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotorB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double currVel = flywheelMotorA.getVelocity();
        double err = currVel - currentTargetVelocity;

        telemetry.addLine("Current Target Velocity: " + currentTargetVelocity);
        telemetry.addLine("Current Step Size: " + stepSizes[stepIndex]);
        telemetry.addLine("Current Step Index: " + stepIndex);
        telemetry.addData("Current Velocity: ", currVel);
        telemetry.addData("Error: ", err);
        telemetry.addData("PIDF Coefficients", flywheelMotorA.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        telemetry.addData("Compression: ", flywheelCompressionPower);
    }
}
