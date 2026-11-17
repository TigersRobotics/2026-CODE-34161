package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;

public class UptakeController {
    private static DcMotor uptakeMotor;
    private static double power;
    private static boolean isInitialized;


    public static void init(HardwareMap hardwareMap, boolean actuallyEnabled) {
        if(actuallyEnabled) {
            uptakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.UPTAKE_MOTOR_NAME);
            uptakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            isInitialized = true;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public static void update(double power) {
        if(isInitialized) {
            UptakeController.power = power;
            uptakeMotor.setPower(power);
        }
    }

    public static double getPower() {
        if(!isInitialized) return 0;
        return  power;
    }

    public static void stop() {
        isInitialized = false;
        uptakeMotor = null;
        power = 0;
    }
}
