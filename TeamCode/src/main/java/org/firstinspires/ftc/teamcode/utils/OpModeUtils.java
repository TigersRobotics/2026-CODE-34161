package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.OpModeMethods;

import java.util.function.Consumer;

public class OpModeUtils {
    public static void initAll(HardwareMap hardwareMap) {
        for (Runnable runnable: OpModeMethods.NO_PARAM_INIT) runnable.run();
        for (Consumer<HardwareMap> consumer: OpModeMethods.HARDWARE_MAP_INIT) consumer.accept(hardwareMap);
    }
    public static void updateSome(){
        for (Runnable runnable: OpModeMethods.NO_PARAM_UPDATE) runnable.run();
    }
}
