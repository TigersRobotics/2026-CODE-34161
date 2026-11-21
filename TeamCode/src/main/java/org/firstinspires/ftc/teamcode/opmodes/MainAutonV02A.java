package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathGenV3;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "V2A - Timed Based")
public class MainAutonV02A extends LinearOpMode {
    // this moves to the next action using a timer
    // less error-prone
    GoBildaOdometry odometry;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Timer.init();
        TimedTelemetry.init(telemetry);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        follower = Constants.create(hardwareMap);
        PathGenV3.init(odometry, follower, true);
        TimedTelemetry.update();
        telemetry.update();

        waitForStart();
        Timer.reset();
        Timer.update();
        PathGenV3.goShoot(PathGenV3.Hive.NEAREST);
        while (Timer.getTime(TimeUnit.MILLISECONDS) < 10000) {
            if (!opModeIsActive()) return;
            update();
        }
        // Shoot TODO add shoot here
        while (Timer.getTime(TimeUnit.MILLISECONDS) < 20000) {
            if (!opModeIsActive()) return;

            // shooter update (if needed)
            update();
        }

        PathGenV3.goPark();
        while (opModeIsActive()) {
            update();
        }

    }

    public void update() {
        Timer.update();
        PathGenV3.update();
        TimedTelemetry.update();
        telemetry.update();
    }
}
