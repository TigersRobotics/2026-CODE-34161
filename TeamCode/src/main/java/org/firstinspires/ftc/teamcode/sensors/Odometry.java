package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class Odometry {
    private final double TICKS_PER_REV = 2048.0;

    private DcMotor horzMotor; ///This uses CYPHER through bore encoder things from studica
    private DcMotor lateralMotor;
    private double x, y, heading; ///0 degrees is forwards
    private IMU imu;
    private AngleUnit angleUnit = AngleUnit.DEGREES;
    private DistanceUnit distanceUnit = DistanceUnit.CM;

    private double headingOffset = 0;
    public Odometry(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);


        horzMotor = hardwareMap.get(DcMotor.class, "uptake");
        lateralMotor = hardwareMap.get(DcMotor.class, "intake");

        horzMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horzMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lateralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TimedTelemetry.addLine("Positioning initialized! :D", 1000);
    }


    double prevHorz, horzCounts;
    double prevLat, latCounts;


    public void update() {
        prevHorz = horzCounts;
        horzCounts = horzMotor.getCurrentPosition();
        double deltaHorz = horzCounts-prevHorz;
        double horzRevs = deltaHorz/TICKS_PER_REV * Math.PI * RobotSpecs.ODO_WHEELS_DIA_CM;

        prevLat = latCounts;
        latCounts = lateralMotor.getCurrentPosition();
        double deltaLat = latCounts - prevLat;
        double latRevs = deltaLat/TICKS_PER_REV * Math.PI * RobotSpecs.ODO_WHEELS_DIA_CM;


        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles() ;
        double raw = yawPitchRollAngles.getYaw(angleUnit) + headingOffset;
        heading = wrapDeg(raw);


        x += horzRevs*Math.cos(Math.toRadians(heading))   - latRevs*Math.sin(Math.toRadians(heading));
        y += horzRevs*Math.sin(Math.toRadians(heading))   + latRevs*Math.cos(Math.toRadians(heading));


    }

    public Pose2D getPose() {
        return new Pose2D(distanceUnit, x, y, angleUnit, heading);
    }

    public double getX(DistanceUnit unit) {
        return toUnit(x, unit);
    }

    public double getY(DistanceUnit unit) {
        return toUnit(y, unit);
    }

    public double getHeading(AngleUnit unit) {
        return toUnit(heading, unit);
    }

    private double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }


    public void reset() {
        x = 0;
        y = 0;
        heading = 0;
        headingOffset = 0;
        imu.resetYaw();

        horzMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horzMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lateralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TimedTelemetry.addLine("You reset me! Hmmph", 1000);
    }


    private double toUnit(double distance, DistanceUnit distanceUnit) {
        switch (distanceUnit) {
            case CM: return distance;
            case MM: return distance*10;
            case INCH: return distance/2.54;
            case METER: return distance/100.0;
        }
        return distance;
    }

    private double toUnit(double angle, AngleUnit angleUnit) {
        switch (angleUnit) {
            case DEGREES: return angle;
            case RADIANS: return Math.toRadians(angle);
        }
        return angle;
    }

    public void setPose(Pose2D newPose) {
        x = newPose.getX(distanceUnit);
        y = newPose.getY(distanceUnit);
        double newHeading = newPose.getHeading(AngleUnit.DEGREES);
        double raw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        headingOffset = newHeading - raw;
        heading = wrapDeg(newHeading);
        TimedTelemetry.addLine("Position Updated! :D", 1000);
    }

}