package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class TimedTelemetry {
    public static ArrayList<TelemetryLine> getTelemetryList(){
        return telemetryList;
    }
    static Telemetry telemetry;
    private static final ArrayList<TelemetryLine> telemetryList= new ArrayList<>();

    public static void init(Telemetry telemetry){
        TimedTelemetry.telemetry = telemetry;
    }

    public static void update(){
        telemetryList.removeIf(line -> !line.update());
    }

    public static void addLine(String caption, int timeMillis, boolean forever){
        telemetryList.add(new TelemetryLine(timeMillis, caption).setForever(forever));
    }
    public static void addLine(String captions, int timeMillis){
        addLine(captions, timeMillis, false);
    }
    public static void addLineForever(String captions){
        addLine(captions, 10000, true);
    }
    public static void addLine(String caption){
        addLine(caption, 1, false);
    }

    public static void addData(String caption, Object value, int timeMillis, boolean forever){
        telemetryList.add(new TelemetryLine(timeMillis, caption, value).setForever(forever));
    }

    public static void addDataForever(String caption, Object value){
        addData(caption, value, 10000, true);
    }

    public static void addData(String caption, Object value, int timeMillis){
        addData(caption, value, timeMillis, false);
    }

    public static void addData(String caption, Object value){
        addData(caption, value, 1, false);
    }

    public static void addData(String caption, String format, int timeMillis, boolean forever, Object... args){
        telemetryList.add(new TelemetryLine(timeMillis, caption, format, args).setForever(forever));
    }
    public static void addDataForever(String captions, String format, Object... args){
        addData(captions, format, 1000, true, args);
    }
    public static void addData(String caption, String format, int timeMillis, Object... args){
        addData(caption, format, timeMillis, false, args);
    }
    public static void addData (String caption, String format, Object... args){
        addData(caption, format, 1, false, args);
    }

}

class TelemetryLine {
    private static ElapsedTime timer = new ElapsedTime();
    boolean forever = false;
    double timeMillis;
    String caption;
    String format;
    Object value = null;
    ArrayList<Object> args;
    enum LineType{
        STRING,
        CAPTIONED_OBJECT,
        FORMATED_OBJECT_LIST

    }
    LineType type;

    public TelemetryLine(int timeMillis, String caption) {
        this.timeMillis = timeMillis;
        this.caption = caption;
        type = LineType.STRING;
    }

    public TelemetryLine(int timeMillis, String caption, Object value) {
        this.timeMillis = timeMillis;
        this.caption = caption;
        this.value = value;
        type= LineType.CAPTIONED_OBJECT;
    }
    public TelemetryLine(int timeMillis, String caption, String format, Object... args) {
        this.timeMillis = timeMillis;
        this.caption = caption;
        this.format = format;
        this.args = new ArrayList<>(List.of(args));
        type = LineType.FORMATED_OBJECT_LIST;
    }

    public TelemetryLine setForever(boolean forever){
        this.timeMillis = 10000;
        this.forever = forever;
        return this;
    }

    public boolean update(){

        if (timeMillis >=1){
            if (!forever) timeMillis -= timer.time(TimeUnit.MILLISECONDS);
        } else return false;

        if(type == LineType.STRING){
            TimedTelemetry.telemetry.addLine(caption);
        } else if (type == LineType.CAPTIONED_OBJECT) {
            TimedTelemetry.telemetry.addData(caption, value);
        } else if (type==LineType.FORMATED_OBJECT_LIST){
            TimedTelemetry.telemetry.addData(caption, format, args);
        }
        timer.reset();
        return true;
    }

    @NonNull
    @Override
    public String toString(){
        return this.caption+" Time: " + this.timeMillis;
    }
}
