package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class OpModeMethods {
    public static ArrayList<Runnable> NO_PARAM_INIT = new ArrayList<>(
            List.of(
                    Timer::init
            )
    );
    public static ArrayList<Consumer<HardwareMap>> HARDWARE_MAP_INIT = new ArrayList<>(
            List.of(

            )
    );

    public static ArrayList<Runnable> NO_PARAM_UPDATE = new ArrayList<>(
            List.of(
                    Timer::update,
                    TimedTelemetry::update
            )
    );
    // no list of parameterized methods for update, as parameters varies by number and type
}
