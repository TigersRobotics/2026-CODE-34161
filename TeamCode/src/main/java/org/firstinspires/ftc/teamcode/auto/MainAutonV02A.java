package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathGenV3;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.sensors.FieldOdometry;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.Shoot;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "V2A - Timed Based")
public class MainAutonV02A extends LinearOpMode {
    // this moves to the next action using a timer
    // less error-prone
    GoBildaOdometry odometry;
    Follower follower;

    FieldOdometry fieldOdometry;

    @Override
    public void runOpMode() throws InterruptedException {
        Timer.init();
        TimedTelemetry.init(telemetry);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        follower = Constants.create(hardwareMap);
        PathGenV3.init(odometry, follower, true);
        TimedTelemetry.update();
        telemetry.update();

        fieldOdometry = new FieldOdometry(odometry);
        fieldOdometry.zero(-72 + 6.5, -72 + 9, 90);


        PollenTurretController.init(hardwareMap);
        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, 0.0, TuningConstants.FLYWHEEL_D_VALUE, TuningConstants.FLYWHEEL_F_VALUE);
        PollenTurretController.setTargetVel(1255);
        SingleIntakeController.init(hardwareMap);

        SingleIntakeController.update(0.2);

        waitForStart();
        Timer.reset();
        Timer.update();
        PathGenV3.goShoot(PathGenV3.Hive.RIGHT_HIVE);
        while (Timer.getTime(TimeUnit.MILLISECONDS) < 10000) {
            if (!opModeIsActive()) return;
            PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, TuningConstants.FLYWHEEL_I_VALUE, TuningConstants.FLYWHEEL_D_VALUE, TuningConstants.FLYWHEEL_F_VALUE);
            PollenTurretController.update();
            update();
        }
        // Shoot TODO add shoot here
        while (Timer.getTime(TimeUnit.MILLISECONDS) < 20000) {
            if (!opModeIsActive()) return;
            Shoot.shoot(hardwareMap, true);
            PollenTurretController.aim(1, fieldOdometry);
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
        PollenTurretController.update();
        odometry.update();
    }
}
