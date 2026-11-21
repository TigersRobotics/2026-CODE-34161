package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

@TeleOp
public class PIDFTuningAHJDIOjAIOWJFIOAIO extends OpMode {
    double P = 118.5, I = 0, D = -0.4, F = 15.1;
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
    double sentP = Double.NaN, sentI = Double.NaN, sentD = Double.NaN, sentF = Double.NaN;
    boolean iArmed = false;
    double iArmTarget = Double.NaN;
    long speedCommandAtNs;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        hood = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
        uptake = hardwareMap.get(DcMotor.class, HardwareNames.UPTAKE_MOTOR_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        setHoodAngle(angle);
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
        // Re-arm below 0.4 so the stick does not have to return all the way to center.
        // A stick resting above 0.75 would otherwise lock that direction after one step.
        if(gamepad1.right_stick_y > 0.75) {
            if(!justTurnedUpI) I += stepSize[stepIndex];
            justTurnedUpI = true;
        } else if(gamepad1.right_stick_y < 0.4) {
            justTurnedUpI = false;
        }
        if(gamepad1.right_stick_y < -0.75) {
            if(!justTurnedDownI) I -= stepSize[stepIndex];
            justTurnedDownI = true;
        } else if(gamepad1.right_stick_y > -0.4) {
            justTurnedDownI = false;
        }

        if(gamepad1.left_stick_y > 0.75) {
            if(!justTurnedUpD) D += stepSize[stepIndex];
            justTurnedUpD = true;
        } else if(gamepad1.left_stick_y < 0.4) {
            justTurnedUpD = false;
        }
        if(gamepad1.left_stick_y < -0.75) {
            if(!justTurnedDownD) D -= stepSize[stepIndex];
            justTurnedDownD = true;
        } else if(gamepad1.left_stick_y > -0.4) {
            justTurnedDownD = false;
        }

        if(gamepad1.leftStickButtonWasPressed()) isNormalSpeed = !isNormalSpeed;

        double targetedSpeed = isNormalSpeed ? targetSpeed : lowSpeed;
        double error = targetedSpeed - flywheel.getVelocity();

        // Only the big low/high speed switch restarts the spin-up hold. Bumper steps of 5
        // must not turn I back off, or tapping the speed up leaves positive I dead.
        if (Double.isNaN(iArmTarget) || Math.abs(targetedSpeed - iArmTarget) > 100) {
            iArmed = false;
            iArmTarget = targetedSpeed;
            speedCommandAtNs = System.nanoTime();
        }
        if (!iArmed && (Math.abs(error) < 150 || System.nanoTime() - speedCommandAtNs > 2_000_000_000L)) {
            iArmed = true;
        }
        double appliedI = iArmed ? I : 0;

        if (P != sentP || appliedI != sentI || D != sentD || F != sentF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, appliedI, D, F, MotorControlAlgorithm.PIDF));
            sentP = P;
            sentI = appliedI;
            sentD = D;
            sentF = F;
        }

        flywheel.setVelocity(targetedSpeed);

        if(gamepad2.circleWasPressed()) angle += TurretConstants.HOOD_STEP_DEG;
        if(gamepad2.triangleWasPressed()) angle -= TurretConstants.HOOD_STEP_DEG;
        angle = UtilMethods.clip(angle, TurretConstants.HOOD_MIN_DEG, TurretConstants.HOOD_MAX_DEG);

        setHoodAngle(angle);

        telemetry.addData("P: ", P);
        telemetry.addData("I", I);
        telemetry.addData("I applied", appliedI);
        telemetry.addData("I stick", gamepad1.right_stick_y);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("Target Velocity:", targetSpeed);
        telemetry.addData("Step Size: ", stepSize[stepIndex]);
        telemetry.addData("Current Velocity: ", flywheel.getVelocity());
        telemetry.addData("Error: ", error);
        telemetry.addData("Hood Angle: ", angle);
        telemetry.addData("Hood Servo: ", hoodPosition);



        telemetry.update();
    }
    double hoodPosition;

    public void setHoodAngle(double degrees) {
        if (hood == null || Double.isNaN(degrees)) return;

        hoodPosition = UtilMethods.clip((degrees - TurretConstants.HOOD_MIN_DEG) / 20.0, 0, 1);
        hood.setPosition(hoodPosition);
    }

}
