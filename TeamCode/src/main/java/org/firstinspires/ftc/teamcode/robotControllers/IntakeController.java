package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;

@Deprecated

public class IntakeController {
    private final DcMotor intakeMotorA;
    private final DcMotor intakeMotorB;

    private double power;

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.intakeMotorA = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_MOTOR_NAME);
        this.intakeMotorB = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE_MOTOR_NAME);

        intakeMotorA.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorB.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("INTAKE INITIALIZED");
    }

    public double getPower() {
        return power;
    }
    public void setPower(double power) {
        this.power = power;
        intakeMotorA.setPower(power);
        intakeMotorB.setPower(power);
    }


}
