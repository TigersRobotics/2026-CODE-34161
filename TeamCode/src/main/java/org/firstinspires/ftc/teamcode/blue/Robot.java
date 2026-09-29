package org.firstinspires.ftc.teamcode.blue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.drive.Odometry;
import org.firstinspires.ftc.teamcode.blue.drive.Pose;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.mechanisms.BallSensor;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;

import java.util.List;

public class Robot {
    public final SwerveDrive drive;
    public final Limelight limelight;
    public final Intake intake;
    public final Launcher launcher;
    public Odometry odometry;
    public Lift lift;
    public BallSensor ballSensor;

    private final List<LynxModule> hubs;
    private final VoltageSensor battery;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    public Robot(HardwareMap hardwareMap, boolean needOdometry) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drive = new SwerveDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        if (needOdometry || hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint") != null) {
            odometry = new Odometry(hardwareMap);
            drive.useOdometry(odometry);
        }
        if (hardwareMap.tryGet(DcMotorEx.class, "lift") != null) {
            lift = new Lift(hardwareMap);
        }
        if (hardwareMap.tryGet(NormalizedColorSensor.class, "ballColor") != null) {
            ballSensor = new BallSensor(hardwareMap);
        }

        battery = hardwareMap.voltageSensor.iterator().next();
        drive.cacheHeading(true);
    }

    public void update() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        if (odometry != null) odometry.update();
        drive.updateHeading();
        limelight.update(drive.getHeadingDegrees());

        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
    }

    public Pose getPose() {
        return odometry == null ? null : odometry.getPose();
    }

    public double getBattery() {
        return battery.getVoltage();
    }

    public double getLoopMs() {
        return loopMs;
    }

    public void stop() {
        drive.stop();
        intake.stop();
        launcher.stop();
        if (lift != null) lift.stop();
        limelight.stop();
    }
}
