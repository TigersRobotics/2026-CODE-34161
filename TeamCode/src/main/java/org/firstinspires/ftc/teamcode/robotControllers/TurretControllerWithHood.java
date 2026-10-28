package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Deprecated
public class TurretControllerWithHood {
    private final String SERVO_LEFT = "0";
    private final String SERVO_RIGHT = "1";
    private final Servo leftServo;
    private final Servo rightServo;
    private Servo hoodServo;
    private final DcMotorEx flywheel;
    private double flywheelMaxVel = 0.0;
    private double currentVelocity;

    private double flywheelPos = 0.0; ///This is in CM
    private Servo flywheelPosServo;

    public TurretControllerWithHood(HardwareMap hardwareMap, Telemetry telemetry) {
        leftServo = hardwareMap.get(Servo.class, SERVO_LEFT);
        rightServo = hardwareMap.get(Servo.class, SERVO_RIGHT);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheelPosServo = hardwareMap.get(Servo.class, "flywheelPos");

        telemetry.addLine("TURRET CONTROLLER INITIALIZED");
    }

    public double getFlywheelPos() {
        return flywheelPos;
    }
    public void setFlywheelPos(double pos) {
        flywheelPosServo.setPosition(pos);
        flywheelPos = pos;
    }


    /// [...](https://docs.revrobotics.com/first-global/fgc-pid)
    /// This is just for tuning, not the actual tuned thing yet
    public void updateFlyWheel(Telemetry telemetry) {
        flywheel.setPower(1.0);

        currentVelocity = flywheel.getVelocity();

        if(currentVelocity > flywheelMaxVel) {
            flywheelMaxVel = currentVelocity;
        }

        telemetry.addLine("Current Velocity: " + currentVelocity);
        telemetry.addLine("Max Velocity: " + flywheelMaxVel);

    }
    public void setTurretPosition(double angle, boolean isRadians) {
        double newAngle = angle;
        if(!isRadians) {
            newAngle = Math.toRadians(angle);
        }

        setPositionRads(leftServo, newAngle);
        setPositionRads(rightServo, newAngle);
    }

    public void setHoodAngle(double angle, boolean isRadians) {
        double newAngle = angle;
        if(!isRadians) {
            newAngle = Math.toRadians(angle);
        }

        setPositionRads(hoodServo, newAngle);
    }

    private void setPositionRads(Servo servo, double angle) {
     servo.setPosition(angle/Math.PI*2);
    }
}
