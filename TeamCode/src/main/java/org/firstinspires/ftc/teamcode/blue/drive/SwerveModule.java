package org.firstinspires.ftc.teamcode.blue.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.PID;

public class SwerveModule {
    public final String name;

    private final DcMotorEx drive;
    private final CRServo turn;
    private final AnalogInput encoder;
    private final PID pid;

    private double offset;
    private double targetAngle = 0;
    private double lastSpeed = 0;

    public SwerveModule(HardwareMap hardwareMap, String name, double offset, boolean driveReversed) {
        this.name = name;
        this.offset = offset;

        drive = hardwareMap.get(DcMotorEx.class, name + "Drive");
        turn = hardwareMap.get(CRServo.class, name + "Turn");
        encoder = hardwareMap.get(AnalogInput.class, name + "Enc");

        drive.setDirection(driveReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turn.setDirection(BlueConstants.TURN_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        pid = new PID(BlueConstants.TURN_P, BlueConstants.TURN_I, BlueConstants.TURN_D);
    }

    public double getVoltage() {
        return encoder.getVoltage();
    }

    public double getRawAngle() {
        double angle = encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI;
        return BlueConstants.ENCODER_REVERSED ? 2 * Math.PI - angle : angle;
    }

    public double getAngle() {
        return AngleUnit.normalizeRadians(getRawAngle() - offset);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getSpeed() {
        return lastSpeed;
    }

    public double getDriveVelocity() {
        return drive.getVelocity();
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void set(double speed, double angle) {
        double error = AngleUnit.normalizeRadians(angle - getAngle());

        if (Math.abs(error) > Math.PI / 2) {
            angle = AngleUnit.normalizeRadians(angle + Math.PI);
            speed = -speed;
            error = AngleUnit.normalizeRadians(angle - getAngle());
        }

        if (Math.abs(AngleUnit.normalizeRadians(angle - targetAngle)) > Math.PI / 4) pid.reset();
        targetAngle = angle;
        lastSpeed = speed * Math.cos(error);

        turn.setPower(turnPower(error));
        drive.setPower(lastSpeed);
    }

    public void hold() {
        double error = AngleUnit.normalizeRadians(targetAngle - getAngle());
        lastSpeed = 0;
        turn.setPower(turnPower(error));
        drive.setPower(0);
    }

    public void setRaw(double drivePower, double turnPower) {
        lastSpeed = drivePower;
        drive.setPower(drivePower);
        turn.setPower(turnPower);
    }

    public void stop() {
        lastSpeed = 0;
        drive.setPower(0);
        turn.setPower(0);
        pid.reset();
    }

    private double turnPower(double error) {
        pid.kP = BlueConstants.TURN_P;
        pid.kI = BlueConstants.TURN_I;
        pid.kD = BlueConstants.TURN_D;

        if (Math.abs(error) < BlueConstants.TURN_TOLERANCE) {
            pid.reset();
            return 0;
        }
        return Range.clip(pid.calculate(error), -1, 1);
    }
}
