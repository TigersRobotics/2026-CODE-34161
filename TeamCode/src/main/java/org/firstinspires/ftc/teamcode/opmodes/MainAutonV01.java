package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathGenV3;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Just go park")
public class MainAutonV01 extends OpMode {
    GoBildaOdometry odometry;
    Follower follower;

    private boolean parkYet = false;

    @Override
    public void init() {
//        PathGenV2.init(odometry, );
        Timer.init();
        TimedTelemetry.init(telemetry);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        follower = Constants.create(hardwareMap);
        PathGenV3.init(odometry, follower, true);
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        Timer.update();
        odometry.update();

        if (Timer.getTime(TimeUnit.MILLISECONDS) > 10000) {
            if (!parkYet) PathGenV3.goPark();
            parkYet = true;
        }


        PathGenV3.update(gamepad1.right_trigger > 0.5);
        TimedTelemetry.update();
        telemetry.update();

    }
}
