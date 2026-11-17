package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FLYWHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_YAW_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MAX_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MIN_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;


public class PollenTurretController {
    private static boolean isInitialized;
    private static DcMotorEx flywheel;
    private static DcMotorEx rotation;
    private static double P, F, targetVel = 0, lowVelocity = 0, highVelocity = 1500;
    private static double yaw = 0; //This is in degrees

    public static void init(HardwareMap hardwareMap, boolean actuallyEnabled) {
        if(actuallyEnabled) {
            flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.REVERSE);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


            rotation = (DcMotorEx) hardwareMap.get(DcMotor.class, TURRET_YAW_MOTOR_NAME);
            rotation.setTargetPosition(0);
            rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///Clockwise should be positive encoder position
            rotation.setDirection(DcMotor.Direction.FORWARD);
            rotation.setPower(1.0);

            isInitialized = true;

            TimedTelemetry.addLine("Pollen Turret Controller Initialized! :D", 1000);

        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }


    public static void update() {
        if(isInitialized) {
            flywheel.setVelocity(targetVel);
            TimedTelemetry.addData("Target Position: ", yaw);
            rotation.setTargetPosition((int) Math.round(yaw * TICKS_PER_REV / 360));
            TimedTelemetry.addData("Current Flywheel Speed: ", flywheel.getVelocity());
        } else TimedTelemetry.addLine("Pollen Turret Updating but not Initialized! >:(");
    }


    ///Sets the angle of the turret in degrees
    public static void setYaw(double yaw) {
        PollenTurretController.yaw = UtilMethods.clip(yaw, MIN_YAW, MAX_YAW);
    }

    public static void setTargetVel(double targetVel) {
        PollenTurretController.targetVel = targetVel;
    }
    public static void setPFCoefficients(double P, double F) {
        PollenTurretController.P = P;
        PollenTurretController.F = F;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public static double getLowVelocity () {
        return lowVelocity;
    }
    public static double getTargetVel() {
        return targetVel;
    }
    public static double getVelocity() {
        return flywheel.getVelocity();
    }

    public static double getCurrentYaw() {
        return rotation.getCurrentPosition();
    }
    public static void stop() {
        flywheel = null;
        rotation = null;
        yaw = 0;
        isInitialized = false;
    }
}
