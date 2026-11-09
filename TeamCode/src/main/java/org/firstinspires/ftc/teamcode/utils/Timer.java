package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Timer {
    private final static ArrayList<TimedTrigger> timedTriggers = new ArrayList<>();
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
        TimedTelemetry.addData("Time: ", timer.time(TimeUnit.MILLISECONDS));
        TimedTelemetry.addData("DTime: ", DTime());
    }
    public static double DTime(){
        return DTime;
    }
    public static void addTrigger(int timeMillis, Runnable runThis){
        timedTriggers.add(new TimedTrigger(timeMillis, runThis));
    }
    private static void updateTriggers(){
        timedTriggers.removeIf(TimedTrigger::update);
    }

}

class TimedTrigger{
    private float millis;
    private final Runnable runThis;
    public TimedTrigger(int timeMillis, Runnable runThis){
        this.millis = timeMillis;
        this.runThis = runThis;
    }

    public boolean update(){
        this.millis = (float) (this.millis - (this.millis - Timer.DTime()));
        if (millis<=0){
            run();
            return true;
        }
        return false;
    }
    private void run(){
        runThis.run();
    }
}