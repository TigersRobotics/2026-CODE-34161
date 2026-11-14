package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FRONT_RIGHT_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.VectorMath;


/**
 * This class is used to control the mecanum drivetrain
 * @author Cole L
 * @version 1.0
 */
public class MecanumDriveTrainController{
    public static boolean isInitialized = false;
    private static DcMotor frontLeft;
    private static DcMotor frontRight;
    private static DcMotor backLeft;
    private static DcMotor backRight;
    private static double frontLeftPower;
    private static double frontRightPower;
    private static double backLeftPower;
    private static double backRightPower;

    public static void init(HardwareMap hardwareMap, boolean actuallyInitialize) {
        if(actuallyInitialize) {

            frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
            frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
            backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR_NAME);
            backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR_NAME);

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            TimedTelemetry.addLine("MECANUM DRIVE TRAIN INITIALIZED");

            isInitialized = true;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public static void stop() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        isInitialized = false;
        frontLeft = null;
        frontRight = null;
        backLeft = null;
        backRight = null;
    }

    /**
     * Updates the drive values
     * @param driveY drive in accordance to the front of the robot
     * @param driveX drive in accordance to the right of the robot
     * does not actually update the movement. updateMovement must be called to update
     */
    public static void update(double driveY, double driveX) {
        update(driveY,driveX, 0);
    }
    /**
     * Starts the movement of the robot.
     * Moves based off the values of driveY, driveX, and turn, set by updateDrive and updateTurn
     */
    public static void update() {
        update(0, 0, 0);
    }

    /**
     * Starts the movement of the robot. Can be used instead of using updateDrive() and updateTurn()
     * @param driveY drive in accordance to the front of the robot
     * @param driveX drive in accordance to the right of the robot
     * @param turn turn in the clockwise direction(0-1)
     */
    public static void update(double driveY, double driveX, double turn) {
        if(isInitialized) {
            TimedTelemetry.addData("driveY: ", driveY);
            TimedTelemetry.addData("driveX: ", driveX);
            TimedTelemetry.addData("turn: ", turn);


            double fl = driveY + driveX + turn;
            double fr = driveY - driveX - turn;
            double bl = driveY - driveX + turn;
            double br = driveY + driveX - turn;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }


            frontLeftPower = fl;
            frontRightPower = fr;
            backRightPower = br;
            backLeftPower = bl;

            // Send calculated power to wheels
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);


        }
    }

}
