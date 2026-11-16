package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

public class TurretUpAngleControllerV2 {
    private static boolean isInitialized = false;
    private static float targetAngle = 0.0074074074f;
    private static float currentAngle = 0.0074074074f;
    private static Servo turretUpAngleServo;
    public static void init(HardwareMap hardwareMap){
        turretUpAngleServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
        TimedTelemetry.addLine("Uptake Controller Initialized");
        isInitialized = true;
        turretUpAngleServo.setPosition(currentAngle);
    }

    public static void update(float power){
        if (!isInitialized){
            TimedTelemetry.addLine("Turret angle up/down controller not initialized! (っ- ‸ - ς)");
            return;
        }
        TimedTelemetry.addData("Power", power);
        TimedTelemetry.addData("Current Angle: ", currentAngle);
        TimedTelemetry.addData("Target Angle: ", targetAngle);
        if (power > 0.5) targetAngle = 0.148148148f;
        else if (power <-0.5 ) targetAngle = 0.0074074074f;
        else targetAngle = (float) turretUpAngleServo.getPosition();
        if (targetAngle> 0.148148148) targetAngle = 0.148148148f;
        else if (targetAngle < 0.0074074074f) targetAngle = 0.0074074074f;
        turretUpAngleServo.setPosition(targetAngle);

        if (targetAngle- currentAngle < 0.00005) return;
        currentAngle = (float) (currentAngle + (Timer.DTime()) / 200*60)*Math.signum(targetAngle-currentAngle);

    }
}
