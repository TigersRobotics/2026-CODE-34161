package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.sensors.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



@TeleOp
public class TurretTestWithPDTuning extends OpMode {
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public IMU imu;
    double currentTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;
    PollenTurretController turretController;
    AprilTagWebcam aprilTagWebcam;
    private double headingOffset = 0;
    private static final double TRACK_GAIN = 0.6;

    @Override
    public void init() {
        turretController = new PollenTurretController(hardwareMap);
         aprilTagWebcam = new AprilTagWebcam(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection));
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        aprilTagWebcam.updateTagDetection();
        turretController.update();

        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles() ;
        double raw = yawPitchRollAngles.getYaw(AngleUnit.DEGREES) + headingOffset;
        double heading = wrapDeg(raw);

        if (gamepad1.aWasPressed()) {
            currentTargetVelocity = (currentTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }
        if(currentTargetVelocity == highVelocity) {
            currentTargetVelocity = lowVelocity;
        } else { currentTargetVelocity = highVelocity; }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.leftTriggerWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if(gamepad1.rightTriggerWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.leftBumperWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if(gamepad1.rightBumperWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()) {
            headingOffset -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()) {
            headingOffset += stepSizes[stepIndex];
        }

        turretController.setPFCoefficients(P, F);

        double currVel = turretController.getVelocity();
        double err = currVel - currentTargetVelocity;


        //Code to rotate the turret(cam mounted on turret)
        AprilTagDetection tag = aprilTagWebcam.getTagByID(24);
        if (tag != null) {
            double bearing = tag.ftcPose.bearing; // degrees
            if (Math.abs(bearing) > 1.0) {        // deadband
                turretController.setYaw(turretController.getCurrentYaw() - TRACK_GAIN * bearing);
            }
        }



        turretController.setTargetVel(currentTargetVelocity);


        telemetry.addLine("Current Target Velocity: " + currentTargetVelocity);
        telemetry.addLine("Current Step Size: " + stepSizes[stepIndex]);
        telemetry.addLine("Current Step Index: " + stepIndex);
        telemetry.addData("Current Velocity: ", currVel);
        telemetry.addData("Error: ", err);
        telemetry.addData("PIDF Coefficients: " + P + ", ",  F);
        aprilTagWebcam.displayTelemetry(telemetry);
    }

    private double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
