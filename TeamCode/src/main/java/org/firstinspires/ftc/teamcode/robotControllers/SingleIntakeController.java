package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class SingleIntakeController {
    private static boolean isInitialized = false;
    private static Telemetry telemetry;
    private static DcMotor intakeMotor;
    private static double power;

    public static void init(HardwareMap hardwareMap, Telemetry telemetry, boolean actuallyInitialize) {
        if(actuallyInitialize) {
            SingleIntakeController.telemetry = telemetry;
            intakeMotor = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_MOTOR_NAME);
            isInitialized = true;
        } else {
            stop();
            TimedTelemetry.addLine("Single Intake Not Initialized!", 5000);
        }
    }

    public static double getPower() {
        return power;
    }

    public static void update(double power) {
        if(isInitialized) {
            SingleIntakeController.power = power;

            intakeMotor.setPower(power);
        } else TimedTelemetry.addLine("Intake being updated but not initialized! >:(");
    }
    public static void stop() {
        intakeMotor = null;
        telemetry = null;
        power = 0;
        isInitialized = false;
    }
}
