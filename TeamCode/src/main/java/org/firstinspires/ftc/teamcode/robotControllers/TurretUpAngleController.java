package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class TurretUpAngleController {
    private static boolean isInitialized = false;
    private static float targetAngle = 0F;
    private static final float MAX_ANGLE = 40f; //Deg
    private static final float SERVO_MAX_ANGLE = 270F; //Deg
    private static final float MIN_ANGLE = 2F; //Deg
    private static Servo turretUpAngleServo;
    public static void init(HardwareMap hardwareMap, Boolean actuallyEnabled) {
        if(actuallyEnabled) {
            turretUpAngleServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
            TimedTelemetry.addLine("Uptake Controller Initialized");
            isInitialized = true;
        }else stop();
    }


    public static void init(HardwareMap hardwareMap){
        init(hardwareMap, true);
    }

    public static void update(float power){
        if (!isInitialized){
            TimedTelemetry.addLine("Turret angle up/down controller not initialized! (っ- ‸ - ς)");
            return;
        }
        TimedTelemetry.addData("Power", power);
        TimedTelemetry.addData("Current Angle: ", turretUpAngleServo.getPosition());
        TimedTelemetry.addData("Target Angle: ", targetAngle);
        if (power > 0.5) targetAngle = MAX_ANGLE /SERVO_MAX_ANGLE;
        else if (power <-0.5 ) targetAngle = MIN_ANGLE /SERVO_MAX_ANGLE;
        else targetAngle = (float) turretUpAngleServo.getPosition();
        if (targetAngle> MAX_ANGLE /SERVO_MAX_ANGLE) targetAngle = MAX_ANGLE /SERVO_MAX_ANGLE;
        else if (targetAngle < 0.05) targetAngle = MIN_ANGLE /SERVO_MAX_ANGLE;
        turretUpAngleServo.setPosition(targetAngle);
    }


    public static void stop() {
        turretUpAngleServo = null;
        isInitialized = false;
        targetAngle = 0F;
    }

}
