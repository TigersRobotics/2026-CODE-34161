package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timer {
    private final static ElapsedTime timer = new ElapsedTime();
    private static double PTime = 0;
    private static double DTime = 0;
    public static double getTime(TimeUnit unit){
        return timer.time(unit);
    }

    public static void init(){
        timer.reset();
    }
    public static void update(){

        DTime = timer.time(TimeUnit.MILLISECONDS) - PTime;
        PTime = timer.time(TimeUnit.MILLISECONDS);
        TimedTelemetry.addData("DTime: ", timer.time(TimeUnit.MILLISECONDS));
    }
    public static double DTime(){
        return DTime;
    }
}
