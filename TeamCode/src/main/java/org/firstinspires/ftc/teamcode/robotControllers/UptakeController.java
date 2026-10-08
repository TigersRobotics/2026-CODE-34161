package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

public class UptakeController {
    private static Servo upTakeServo;
    private boolean uptakeBusy = false;
    private final Telemetry telemetry;
    public UptakeController(HardwareMap hardwareMap, Telemetry telemetry){
        upTakeServo = hardwareMap.get(Servo.class, HardwareNames.UPTAKE_SERVO_NAME);
        this.telemetry = telemetry;
        telemetry.addLine("Uptake Controller Initialized");
    }

    public boolean shoot(){
        if (uptakeBusy) {
            telemetry.addLine("Uptake Busy. If this shows while it is not busy, adjust constants");
            return false;
        }

        upTakeServo.setPosition(RobotSpecs.UPTAKE_SERVO_UP_ANGLE);
        uptakeBusy = true;
        return true;
    }

    public void updateUptake (){
        if (upTakeServo.getPosition() == RobotSpecs.UPTAKE_SERVO_DEFAULT_ANGLE){
            uptakeBusy = false;
        }
        if (upTakeServo.getPosition() == RobotSpecs.UPTAKE_SERVO_UP_ANGLE){
            telemetry.addLine("Uptake Moving down");
            upTakeServo.setPosition(RobotSpecs.UPTAKE_SERVO_DEFAULT_ANGLE);
        }

        if (uptakeBusy){
            telemetry.addLine("Uptake busy");
        } else {
            telemetry.addLine("Uptake free");
        }
    }
}