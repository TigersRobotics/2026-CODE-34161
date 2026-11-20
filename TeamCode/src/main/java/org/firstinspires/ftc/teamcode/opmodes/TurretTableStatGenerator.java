package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.sensors.FieldOdometry;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

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

        boolean running = true;
        while(running) {
            odometry.update();
            MecanumDriveTrainController.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, odometry.getHeading());
            PollenTurretController.update();
            SingleIntakeController.update(toggleIntake ? 1 : 0);

            if(gamepad1.leftBumperWasPressed()) toggleIntake = !toggleIntake;

            if(gamepad1.dpadUpWasPressed()) {
                print();
                data.clear();
                angle += 0.5;
            }
            if(gamepad1.dpadDownWasPressed()) {
                print();
                data.clear();

                angle -= 0.5;
            }

            if(gamepad1.triangleWasPressed()) {
                flywheelSpeed += 5;
            }
            if(gamepad1.squareWasPressed()) {
                flywheelSpeed -= 5;
            }

            UptakeController.update(gamepad1.right_trigger - gamepad1.left_trigger);

            PollenTurretController.setTargetVel(flywheelSpeed);
            PollenTurretController.setHoodAngle(angle);

            if(gamepad1.circleWasPressed()) {
                ///Add the current x, y, angle, and flywheel speed to the list
                double x = odometry.getX();
                double y = odometry.getY();

                ArrayList<String> row = new ArrayList<>(List.of(String.valueOf(angle), String.valueOf(x), String.valueOf(y), String.valueOf(flywheelSpeed)));
                data.add(row);
            }
            if(gamepad2.crossWasPressed()) running = false;
            TimedTelemetry.addData("Flywheel Speed: ", flywheelSpeed);
            TimedTelemetry.addData("X: ", odometry.getX());
            TimedTelemetry.addData("Y: ", odometry.getY());
            TimedTelemetry.addData("Hood Angle: ", angle);


            TimedTelemetry.update();
            telemetry.update();
        }

        print();


    }
    void print() {
        try (PrintWriter writer = new PrintWriter(new FileWriter( new File(AppUtil.FIRST_FOLDER.getName() + angle + "shots.csv")), true)) {
            for (List<String> row : data) {
                writer.println(String.join(",", row));
                TimedTelemetry.addData("Added List",  row.toString(), 10000);
            }

            TimedTelemetry.addLine("Table Generated Successfully");


        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }
}


