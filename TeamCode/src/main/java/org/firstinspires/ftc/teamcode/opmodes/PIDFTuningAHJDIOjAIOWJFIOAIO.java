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
    PIDFCoefficients hubPidf = new PIDFCoefficients();
    long lastReadbackNs = 0;
    double sentP = Double.NaN, sentI = Double.NaN, sentD = Double.NaN, sentF = Double.NaN;

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

        // Read back what the hub is really running. Read right after the values change and
        // twice a second otherwise; each read is a round trip to the hub, so not every loop.
        boolean changed = P != sentP || I != sentI || D != sentD || F != sentF;
        if (changed || System.nanoTime() - lastReadbackNs > 500_000_000L) {
            hubPidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            lastReadbackNs = System.nanoTime();
            sentP = P; sentI = I; sentD = D; sentF = F;
        }

        if(gamepad1.leftStickButtonWasPressed()) isNormalSpeed = !isNormalSpeed;

        double targetedSpeed = isNormalSpeed ? targetSpeed : lowSpeed;

        flywheel.setVelocity(targetedSpeed);

        double velocity = flywheel.getVelocity();
        double error = targetedSpeed - velocity;

        setHoodAngle(angle);



        telemetry.addData("P: ", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("Target Velocity:", targetSpeed);
        // targetSpeed is only sent while normal speed is on (left stick button); otherwise lowSpeed is.
        telemetry.addData("Commanded Velocity", "%.0f (%s)", targetedSpeed, isNormalSpeed ? "normal" : "LOW");
        telemetry.addData("Step Size: ", stepSize[stepIndex]);
        telemetry.addData("Current Velocity: ", velocity);
        telemetry.addData("Error: ", error);
        telemetry.addData("Hub PIDF", "P %.3f I %.3f D %.3f F %.3f %s",
                hubPidf.p, hubPidf.i, hubPidf.d, hubPidf.f, hubPidf.algorithm);
        telemetry.addData("Hub matches tuner", Math.abs(hubPidf.p - P) < 0.001 && Math.abs(hubPidf.i - I) < 0.001
                && Math.abs(hubPidf.d - D) < 0.001 && Math.abs(hubPidf.f - F) < 0.001);
        telemetry.addData("Mode", flywheel.getMode());
        telemetry.addData("Motor", "port %d, %s", flywheel.getPortNumber(), flywheel.getMotorType().getName());
        // Power the F term alone asks for. If velocity tracks this open-loop guess instead of the
        // target, P and I are not acting on the hub.
        telemetry.addData("F-only power", "%.3f", F * targetedSpeed / 32767.0);



        telemetry.update();
    }
    public void setHoodAngle(double degrees) {
        if (hood == null || Double.isNaN(degrees)) return;

        double position = UtilMethods.clip((degrees - TurretConstants.HOOD_MIN_DEG) / HOOD_SERVO_RANGE_DEG, 0, 1);
        hood.setPosition(position);
    }

}
