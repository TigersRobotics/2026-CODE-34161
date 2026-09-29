package org.firstinspires.ftc.teamcode.blue.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

public class Launcher {
    private final DcMotorEx flywheel;
    private final Servo hood;
    private final CRServo feeder;

    private double targetRpm = 0;
    private double hoodPosition = 0.5;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "launcher");
        hood = hardwareMap.get(Servo.class, "hood");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyPidf();

        setHood(hoodPosition);
    }

    public void applyPidf() {
        flywheel.setVelocityPIDFCoefficients(BlueConstants.LAUNCHER_P, BlueConstants.LAUNCHER_I,
                BlueConstants.LAUNCHER_D, BlueConstants.LAUNCHER_F);
    }

    public void setRawPower(double power) {
        targetRpm = 0;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(power);
    }

    public double getTicksPerSecond() {
        return flywheel.getVelocity();
    }

    public void setRpm(double rpm) {
        targetRpm = rpm;
        if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        flywheel.setVelocity(rpm / 60.0 * BlueConstants.LAUNCHER_TICKS_PER_REV);
    }

    public double getRpm() {
        return flywheel.getVelocity() / BlueConstants.LAUNCHER_TICKS_PER_REV * 60.0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public boolean isReady() {
        return targetRpm > 0 && Math.abs(getRpm() - targetRpm) < BlueConstants.LAUNCHER_RPM_TOLERANCE;
    }

    public void setHood(double position) {
        hoodPosition = Range.clip(position, BlueConstants.HOOD_MIN, BlueConstants.HOOD_MAX);
        hood.setPosition(hoodPosition);
    }

    public double getHood() {
        return hoodPosition;
    }

    public void aimFor(double distance) {
        setRpm(ShotTable.rpm(distance));
        setHood(ShotTable.hood(distance));
    }

    public void feed() {
        feeder.setPower(BlueConstants.FEED_POWER);
    }

    public void unjam() {
        feeder.setPower(-BlueConstants.FEED_POWER);
    }

    public void stopFeed() {
        feeder.setPower(0);
    }

    public void idle() {
        if (BlueConstants.LAUNCHER_IDLE_RPM > 0) {
            setRpm(BlueConstants.LAUNCHER_IDLE_RPM);
            stopFeed();
        } else {
            stop();
        }
    }

    public void stop() {
        targetRpm = 0;
        if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        flywheel.setPower(0);
        stopFeed();
    }
}
