package org.firstinspires.ftc.teamcode.blue.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class DataLogger {
    public static final File LOG_FOLDER = new File(AppUtil.ROOT_FOLDER, "blue_logs");

    private final File file;
    private FileWriter writer;
    private int rows = 0;

    public DataLogger(String name, String... columns) {
        LOG_FOLDER.mkdirs();
        String stamp = new SimpleDateFormat("MMdd_HHmmss", Locale.US).format(new Date());
        file = new File(LOG_FOLDER, name + "_" + stamp + ".csv");

        try {
            writer = new FileWriter(file);
            writeLine(columns);
        } catch (IOException e) {
            writer = null;
        }
    }

    public void log(Object... values) {
        writeLine(values);
        rows++;
    }

    private void writeLine(Object[] values) {
        if (writer == null) return;

        StringBuilder line = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) line.append(',');
            line.append(values[i]);
        }
        line.append('\n');

        try {
            writer.write(line.toString());
        } catch (IOException e) {
            writer = null;
        }
    }

    public int getRows() {
        return rows;
    }

    public boolean isWorking() {
        return writer != null;
    }

    public String getPath() {
        return file.getAbsolutePath();
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
        }
        writer = null;
    }
}
