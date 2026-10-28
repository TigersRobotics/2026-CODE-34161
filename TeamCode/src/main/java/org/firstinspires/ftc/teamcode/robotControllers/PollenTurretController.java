package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FLYWHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_YAW_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MAX_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MIN_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;


public class PollenTurretController {
    private DcMotorEx flywheel;
    private DcMotorEx rotation;

    private double P, F, targetVel = 0, lowVelocity = 0, highVelocity = 1500;
    private double yaw = 0; //This is in degrees
    private double currentVel;

    public PollenTurretController(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rotation = hardwareMap.get(DcMotorEx.class, TURRET_YAW_MOTOR_NAME);
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///Clockwise should be positive encoder position
        rotation.setDirection(DcMotor.Direction.FORWARD);
        rotation.setPower(1.0);


        TimedTelemetry.addLine("Pollen Turret Controller Initialized! :D", 1000);
    }

    public void update() {
        flywheel.setVelocity(targetVel);

        if(currentVel == highVelocity) {
            targetVel = lowVelocity;
        } else { currentVel = highVelocity; }

        rotation.setTargetPosition((int) Math.round(yaw * TICKS_PER_REV/360));
    }


    ///Sets the angle of the turret in degrees
    public void setYaw(double yaw) {
        this.yaw = Math.max(MIN_YAW, Math.min(MAX_YAW, yaw));
    }

    public void setTargetVel(double targetVel) {
        this.targetVel = targetVel;
    }
    public void setPFCoefficients(double P, double F) {
        this.P = P;
        this.F = F;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public double getLowVelocity () {
        return lowVelocity;
    }
    public double getTargetVel() {
        return targetVel;
    }
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getCurrentYaw() {
        return rotation.getCurrentPosition();
    }
}
