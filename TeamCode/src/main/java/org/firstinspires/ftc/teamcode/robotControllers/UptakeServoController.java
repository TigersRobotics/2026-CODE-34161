package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class UptakeServoController {
    private static boolean isInitialized = false;
    private static Servo upTakeServo;
    private static boolean uptakeBusy = false;
    private static Telemetry telemetry;
    public static void init(HardwareMap hardwareMap, Telemetry telemetry, boolean actuallyInitialize){
        if(actuallyInitialize) {
            upTakeServo = hardwareMap.get(Servo.class, HardwareNames.UPTAKE_SERVO_NAME);
            UptakeServoController.telemetry = telemetry;
            telemetry.addLine("Uptake Controller Initialized");
            isInitialized = true;
        }
    }

    public static boolean shoot(){
        if(isInitialized) {
            if (uptakeBusy) {
                telemetry.addLine("Uptake Busy. If this shows while it is not busy, adjust constants");
                return false;
            }

            upTakeServo.setPosition(RobotSpecs.UPTAKE_SERVO_UP_ANGLE);
            uptakeBusy = true;
            return true;
        } else return false;
    }

    public static void update() {
        if(isInitialized) {
            if (upTakeServo.getPosition() == RobotSpecs.UPTAKE_SERVO_DEFAULT_ANGLE) {
                uptakeBusy = false;
            }
            if (upTakeServo.getPosition() == RobotSpecs.UPTAKE_SERVO_UP_ANGLE) {
                telemetry.addLine("Uptake Moving down");
                upTakeServo.setPosition(RobotSpecs.UPTAKE_SERVO_DEFAULT_ANGLE);
            }

            if (uptakeBusy) {
                telemetry.addLine("Uptake busy");
            } else {
                telemetry.addLine("Uptake free");
            }
        } else TimedTelemetry.addLine("UptakeController being updated but not initialized!");
    }

    public static void stop() {
        isInitialized = false;
        upTakeServo = null;
        uptakeBusy = false;
    }
}