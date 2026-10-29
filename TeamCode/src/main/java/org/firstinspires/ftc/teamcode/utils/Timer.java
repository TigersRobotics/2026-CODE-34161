package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timer {
    private final static ElapsedTime timer = new ElapsedTime();
    public static double getTime(TimeUnit unit){
        return timer.time(unit);
    }

    public static void init(){
        timer.reset();
    }
}
