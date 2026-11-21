package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.sensors.FieldOdometry;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;



@TeleOp(name = "Table Stat Generator")
public class TurretTableStatGenerator extends LinearOpMode {

    List<List<String>> data = new ArrayList<>(new ArrayList<>());
    FieldOdometry odometry;
    boolean toggleIntake;
    double angle = 7;
    double flywheelSpeed = 1300;

    @Override
    public void runOpMode() throws InterruptedException {
        TimedTelemetry.init(telemetry);
        MecanumDriveTrainController.init(hardwareMap);
        PollenTurretController.init(hardwareMap);
        odometry = new FieldOdometry(new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME));
        UptakeController.init(hardwareMap);
        SingleIntakeController.init(hardwareMap);
        odometry.zero(-72 + 6.5, -72 + 9, 90);
        Timer.init();

        PollenTurretController.setPFCoefficients(TuningConstants.FLYWHEEL_P_VALUE, TuningConstants.FLYWHEEL_I_VALUE, TuningConstants.FLYWHEEL_D_VALUE, TuningConstants.FLYWHEEL_F_VALUE);


        boolean running = true;
        waitForStart();

        try {


            while (running) {
                odometry.update();
                MecanumDriveTrainController.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, odometry.getHeading());
                PollenTurretController.update();
                SingleIntakeController.update(toggleIntake ? 1 : 0);

                if (gamepad1.leftBumperWasPressed()) toggleIntake = !toggleIntake;

                if (gamepad2.rightBumperWasPressed()) {
                    print();
                    data.clear();
                    angle += 0.5;
                }
                if (gamepad2.leftBumperWasPressed()) {
                    print();
                    data.clear();

                    angle -= 0.5;
                }

                if (gamepad1.triangleWasPressed()) {
                    flywheelSpeed += 5;
                }
                if (gamepad1.squareWasPressed()) {
                    flywheelSpeed -= 5;
                }

                UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);

                PollenTurretController.setTargetVel(flywheelSpeed);
                PollenTurretController.setHoodAngle(angle);

                if (gamepad1.circleWasPressed()) {
                    ///Add the current x, y, angle, and flywheel speed to the list
                    double x = odometry.getX();
                    double y = odometry.getY();

                    ArrayList<String> row = new ArrayList<>(List.of(String.valueOf(angle), String.valueOf(x), String.valueOf(y), String.valueOf(flywheelSpeed)));
                    data.add(row);
                }
                if (gamepad2.crossWasPressed() || isStopRequested()) running = false;
                TimedTelemetry.addData("Target Flywheel Speed: ", flywheelSpeed);
                TimedTelemetry.addData("Current Flywheel Speed: ", PollenTurretController.getVelocity());
                TimedTelemetry.addData("X: ", odometry.getX());
                TimedTelemetry.addData("Y: ", odometry.getY());
                TimedTelemetry.addData("Hood Angle: ", angle);

                Timer.update();
                TimedTelemetry.update();
                telemetry.update();
            }
        } finally {
            print();
        }

    }
    void print() {
        if (data.isEmpty()) return;
        File file = new File(AppUtil.FIRST_FOLDER, angle + "shots.csv");
        try (PrintWriter writer = new PrintWriter(new FileWriter(file, true))) {  // true = append
            for (List<String> row : data) {
                writer.println(String.join(",", row));
                TimedTelemetry.addData("Data: ", data.toString(), 10000);


            }
        } catch (IOException e) {
            telemetry.addData("Save error", e.getMessage());
            telemetry.update();
        }
    }
}


