package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

@TeleOp
public class PIDFTuningAHJDIOjAIOWJFIOAIO extends OpMode {
    private static final double HOOD_SERVO_RANGE_DEG = 20;
    double P = 0.0, I = 0.0, D = 0.0, F = 0.0;
    DcMotorEx flywheel;
    Servo hood;
    double angle =  20;
    DcMotor uptake;


    double[] stepSize = {100.0, 10.0, 1.0, 0.5, 0.1, 0.05, 0.01, 0.001};
    int stepIndex = 2;
    double targetSpeed = 1440;
    boolean justTurnedUpI = false;
    boolean justTurnedDownI = false;
    boolean justTurnedUpD = false;
    boolean justTurnedDownD = false;
    boolean isNormalSpeed = false;
    double lowSpeed = 900;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
        uptake = hardwareMap.get(DcMotor.class, HardwareNames.UPTAKE_MOTOR_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) stepIndex = (stepIndex + 1) % stepSize.length;
        if (gamepad1.dpadLeftWasPressed())  stepIndex = (stepIndex - 1 + stepSize.length) % stepSize.length;

        if(gamepad1.leftBumperWasPressed()) targetSpeed -= 5;
        if(gamepad1.rightBumperWasPressed()) targetSpeed += 5;

        uptake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad1.circleWasPressed()) P += stepSize[stepIndex];
        if(gamepad1.squareWasPressed()) P -= stepSize[stepIndex];

        if(gamepad1.triangleWasPressed()) F += stepSize[stepIndex];
        if(gamepad1.crossWasPressed()) F -= stepSize[stepIndex];
        if(gamepad1.right_stick_y > 0.75) {
            if(!justTurnedUpI) I += stepSize[stepIndex];
            justTurnedUpI = true;

        } else {
            justTurnedUpI = false;
        }
        if(gamepad1.right_stick_y < -0.75) {
            if(!justTurnedDownI) I -= stepSize[stepIndex];
            justTurnedDownI = true;

        } else {
            justTurnedDownI = false;
        }


        if(gamepad1.left_stick_y > 0.75) {
            if(!justTurnedUpD) D += stepSize[stepIndex];
            justTurnedUpD = true;

        } else {
            justTurnedUpD = false;
        }
        if(gamepad1.left_stick_y < -0.75) {
            if(!justTurnedDownD) D -= stepSize[stepIndex];
            justTurnedDownD = true;

        } else {
            justTurnedDownD = false;
        }

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        if(gamepad1.leftStickButtonWasPressed()) isNormalSpeed = !isNormalSpeed;

        double targetedSpeed = isNormalSpeed ? targetSpeed : lowSpeed;

        flywheel.setVelocity(targetedSpeed);

        double error = targetedSpeed - flywheel.getVelocity();

        if(gamepad2.circleWasPressed()) angle += TurretConstants.HOOD_STEP_DEG;
        if(gamepad2.triangleWasPressed()) angle -= TurretConstants.HOOD_STEP_DEG;
        angle = UtilMethods.clip(angle, TurretConstants.HOOD_MIN_DEG, TurretConstants.HOOD_MAX_DEG);

        setHoodAngle(angle);

        telemetry.addData("P: ", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("Target Velocity:", targetSpeed);
        telemetry.addData("Step Size: ", stepSize[stepIndex]);
        telemetry.addData("Current Velocity: ", flywheel.getVelocity());
        telemetry.addData("Error: ", error);
        telemetry.addData("Hood Angle: ", angle);



        telemetry.update();
    }
    public void setHoodAngle(double degrees) {
        if (hood == null || Double.isNaN(degrees)) return;

        double position = UtilMethods.clip((degrees - TurretConstants.HOOD_MIN_DEG) / HOOD_SERVO_RANGE_DEG, 0, 1);
        hood.setPosition(position);
    }

}
