package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.TimedTelemetry.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;

public class Shoot {
    public static void shoot(HardwareMap hardwareMap, boolean Button){
        UptakeController.init(hardwareMap, Button);
        UptakeController.update(1);
        telemetry.addLine("SHOOT PLEASEEEEE");
    }
}
