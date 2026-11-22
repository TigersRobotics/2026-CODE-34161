package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FRONT_RIGHT_MOTOR_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.Competition;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;


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
    private static final double POWER_WRITE_TOLERANCE = 0.01;
    // The power last sent to each wheel. NaN means nothing has been sent yet.
    private static double frontLeftPower = Double.NaN;
    private static double frontRightPower = Double.NaN;
    private static double backLeftPower = Double.NaN;
    private static double backRightPower = Double.NaN;
    private static boolean isFieldCentric;

    public static void init(HardwareMap hardwareMap, boolean actuallyInitialize) {
        init(hardwareMap, actuallyInitialize, false);
    }

    public static void init(HardwareMap hardwareMap, boolean actuallyInitialize, boolean fieldCentric) {
        if(actuallyInitialize) {

            frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
            frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
            backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR_NAME);
            backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR_NAME);

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            // The statics survive between OpMode runs, so forget what was sent last time
            frontLeftPower = Double.NaN;
            frontRightPower = Double.NaN;
            backLeftPower = Double.NaN;
            backRightPower = Double.NaN;

            TimedTelemetry.addLine("MECANUM DRIVE TRAIN INITIALIZED");

            isFieldCentric = fieldCentric;

            isInitialized = true;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true, true);
    }

    public static void stop() {
        if(frontRight != null) frontRight.setPower(0);
        if(frontLeft != null) frontLeft.setPower(0);
        if(backRight != null) backRight.setPower(0);
        if(backLeft != null) backLeft.setPower(0);

        isInitialized = false;
        frontLeft = null;
        frontRight = null;
        backLeft = null;
        backRight = null;
    }


    /**
     * Starts the movement of the robot.
     * Moves based off the values of driveY, driveX, and turn, set by updateDrive and updateTurn
     */
    public static void update() {
        update(0, 0, 0, 0);
    }

    public static void update(double driveY, double driveX, double turn) {
        update(driveY, driveX, turn, 0);
    }
    /**
     * Starts the movement of the robot. Can be used instead of using updateDrive() and updateTurn()
     * @param driveY drive in accordance to the front of the robot
     * @param driveX drive in accordance to the right of the robot
     * @param turn turn in the clockwise direction(0-1)
     */
    public static void update(double driveY, double driveX, double turn, double heading) {
        if(isInitialized) {
            double headingRad = Math.toRadians(heading);
            double fieldCentricX = driveX;
            double fieldCentricY = driveY;

            if(isFieldCentric) {
                fieldCentricX = driveX * Math.cos(headingRad) + driveY * Math.sin(headingRad);
                fieldCentricY = driveY * Math.cos(headingRad) - driveX * Math.sin(headingRad);
            }

            TimedTelemetry.addData("driveY: ", driveY);
            TimedTelemetry.addData("driveX: ", driveX);
            TimedTelemetry.addData("turn: ", turn);


            double fl = fieldCentricY + fieldCentricX + turn;
            double fr = fieldCentricY - fieldCentricX - turn;
            double bl = fieldCentricY - fieldCentricX + turn;
            double br = fieldCentricY + fieldCentricX - turn;

            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));

            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }


            // Send calculated power to wheels, skipping any wheel whose power has barely moved.
            // Each skipped wheel is one less hub command in the loop.
            if(isWorthSending(fl, frontLeftPower)) {
                frontLeft.setPower(fl);
                frontLeftPower = fl;
            }
            if(isWorthSending(fr, frontRightPower)) {
                frontRight.setPower(fr);
                frontRightPower = fr;
            }
            if(isWorthSending(bl, backLeftPower)) {
                backLeft.setPower(bl);
                backLeftPower = bl;
            }
            if(isWorthSending(br, backRightPower)) {
                backRight.setPower(br);
                backRightPower = br;
            }
        }
    }

    /// Stick noise changes the power by tiny amounts every loop. Anything under 1% is not sent,
    /// except a stop, which always goes through.
    private static boolean isWorthSending(double power, double sentPower) {
        if(Double.isNaN(sentPower)) return true;
        if(power == 0) return sentPower != 0;
        return Math.abs(power - sentPower) >= POWER_WRITE_TOLERANCE;
    }
    private static final boolean driverOrientate = true;
    private static final boolean buttonHold = false;
    private static final double driverOffset = Competition.STARTING_HEADING_DRIVER_OFFSET_RAD;


    public static void toggleDriverOrientation(boolean on){

    }

}
