package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeController {
    private DcMotor flywheel;
    private double power;

    public IntakeController(DcMotor flywheel, Telemetry telemetry) {
        this.flywheel = flywheel;

        telemetry.addLine("FLYWHEEL INITIALIZED");
    }

    public void setPower(double power) {
        this.power = power;
        flywheel.setPower(power);
    }


}
