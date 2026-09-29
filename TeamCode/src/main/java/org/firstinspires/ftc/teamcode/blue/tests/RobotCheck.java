package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Robot Check", group = "Blue Test")
public class RobotCheck extends OpMode {
    String[] modules = {"fl", "fr", "bl", "br"};
    List<String> missing = new ArrayList<>();
    List<String> optionalMissing = new ArrayList<>();

    AnalogInput[] encoders = new AnalogInput[4];
    IMU imu;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;
    VoltageSensor battery;
    List<LynxModule> hubs;

    @Override
    public void init() {
        for (int i = 0; i < 4; i++) {
            need(DcMotorEx.class, modules[i] + "Drive");
            need(CRServo.class, modules[i] + "Turn");
            encoders[i] = need(AnalogInput.class, modules[i] + "Enc");
        }
        imu = need(IMU.class, "imu");
        limelight = need(Limelight3A.class, "limelight");
        need(DcMotorEx.class, "intake");
        need(DcMotorEx.class, "launcher");
        need(Servo.class, "hood");
        need(CRServo.class, "feeder");

        pinpoint = optional(GoBildaPinpointDriver.class, "pinpoint");
        optional(DcMotorEx.class, "lift");
        optional(NormalizedColorSensor.class, "ballColor");

        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(BlueConstants.HUB_LOGO, BlueConstants.HUB_USB)));
        }
        if (limelight != null) limelight.start();

        battery = hardwareMap.voltageSensor.iterator().next();
        hubs = hardwareMap.getAll(LynxModule.class);
    }

    <T> T need(Class<? extends T> type, String name) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) missing.add(name);
        return device;
    }

    <T> T optional(Class<? extends T> type, String name) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) optionalMissing.add(name);
        return device;
    }

    @Override
    public void init_loop() {
        show();
    }

    @Override
    public void loop() {
        show();
    }

    void show() {
        telemetry.addData("Config", missing.isEmpty() ? "ALL GOOD" : "MISSING " + missing);
        if (!optionalMissing.isEmpty()) telemetry.addData("Not set up (ok)", optionalMissing);

        double volts = battery.getVoltage();
        telemetry.addData("Battery", "%.2f V%s", volts, volts < BlueConstants.LOW_BATTERY ? "  CHANGE IT" : "");
        for (LynxModule hub : hubs) {
            telemetry.addData("Hub " + hub.getDeviceName(), "%.2f A", hub.getCurrent(CurrentUnit.AMPS));
        }

        telemetry.addLine();
        for (int i = 0; i < 4; i++) {
            if (encoders[i] == null) continue;
            double v = encoders[i].getVoltage();
            String note = "";
            if (v < 0.02) note = "  unplugged?";
            else if (v > encoders[i].getMaxVoltage() - 0.02) note = "  maxed out?";
            telemetry.addData(modules[i] + " encoder", "%.3f V  %.1f deg%s", v,
                    v / encoders[i].getMaxVoltage() * 360, note);
        }

        if (imu != null) {
            telemetry.addData("IMU yaw", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }

        if (pinpoint != null) {
            pinpoint.update();
            telemetry.addData("Pinpoint", "%s  enc %d / %d", pinpoint.getDeviceStatus(), pinpoint.getEncoderX(), pinpoint.getEncoderY());
        }

        if (limelight != null) {
            telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "NOT CONNECTED");
            if (limelight.isConnected()) {
                telemetry.addData("LL fps / temp", "%.0f / %.0f C", limelight.getStatus().getFps(), limelight.getStatus().getTemp());
                telemetry.addData("LL pipeline", limelight.getStatus().getPipelineIndex());
            }
        }

        telemetry.addLine();
        telemetry.addLine("Turn each wheel by hand and check its encoder moves");
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
