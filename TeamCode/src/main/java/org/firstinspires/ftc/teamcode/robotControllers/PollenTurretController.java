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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;


public class PollenTurretController {
    private static boolean isInitialized;
    private static DcMotorEx flywheel;
    private static DcMotorEx rotation;
    private static double P, F, targetVel = 0;
    private static double yaw = 0; //This is in degrees

    private static Servo hoodServo;

    private static boolean updateYaw = false;
    private static boolean updateVel = false;
    private static double flywheelVel = 0;

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

            hoodServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);

            TimedTelemetry.addLine("Pollen Turret Controller Initialized! :D", 1000);
            updateYaw = false;
            updateVel = false;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }


    public static void update() {
        if(isInitialized) {
            if(updateVel) {
                flywheel.setVelocity(targetVel);
                updateVel = false;
            }
            TimedTelemetry.addData("Target Position: ", yaw);

            if(updateYaw) {
                rotation.setTargetPosition((int) Math.round(yaw * TICKS_PER_REV / 360));
                updateYaw = false;
            }
            flywheelVel = flywheel.getVelocity();
            TimedTelemetry.addData("Current Flywheel Speed: ", flywheelVel);
        } else TimedTelemetry.addLine("Pollen Turret Updating but not Initialized! >:(");
    }


    ///Sets the angle of the turret in degrees
    public static void setYaw(double yaw) {
        updateYaw = true;
        PollenTurretController.yaw = UtilMethods.clip(yaw, MIN_YAW, MAX_YAW);
    }

    public static void setTargetVel(double targetVel) {
        updateVel = true;
        PollenTurretController.targetVel = targetVel;
    }
    public static void setPFCoefficients(double P, double F) {
        if(isInitialized) {
            PollenTurretController.P = P;
            PollenTurretController.F = F;
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
    }

    public static double getTargetVel() {
        return targetVel;
    }
    public static double getVelocity() {
        return flywheelVel;
    }

    public static double getCurrentYaw() {
        return rotation.getCurrentPosition() * 360 / TICKS_PER_REV;
    }
    public static void stop() {
        flywheel.setPower(0);
        rotation.setPower(0);
        hoodServo.setPosition(0);
        flywheel = null;
        rotation = null;
        hoodServo = null;
        yaw = 0;
        isInitialized = false;
    }

    public static void setHoodAngle(double degrees) {
        hoodServo.setPosition(UtilMethods.clip(degrees, 0, 270)/ 270);
    }
}
