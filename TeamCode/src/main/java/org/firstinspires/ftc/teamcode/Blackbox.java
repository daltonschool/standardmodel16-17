package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class Blackbox {
    public static String logFilePath;

    public static void init() {
        logFilePath = getPathToFile();
        saveTextToFile("Log " + getPrettyTime());
        log("STRT", "Logging ready.");
    }

    public static String getPrettyTime() {
        return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(Calendar.getInstance().getTime());
    }

    public static String getPathToFile() {
        return "/sdcard/blackbox/" + getPrettyTime() + ".log";
    }

    public static void saveTextToFile(String text) {
        File logFile = new File(logFilePath);
        if (!logFile.exists()) {
            try {
                logFile.createNewFile();
            } catch (IOException e) {
                // don't have many options at this point, so just print the stack trace
                e.printStackTrace();
            }
        }
        try {
            FileWriter w = new FileWriter(logFile, true);
            w.write(text);
            w.close();
        } catch (IOException e) {
            // well, we can't really log it, so just print the stack trace...
            e.printStackTrace();
        }
    }

    public static void log(String tag, String msg) {
        saveTextToFile("[" + getPrettyTime() + "][" + tag + "] " + msg + "\n");
    }

    public static void log(Exception e) {
        // http://stackoverflow.com/questions/1149703/how-can-i-convert-a-stack-trace-to-a-string
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        e.printStackTrace(pw);
        log("ERR ", sw.toString());
        pw.close();
        try {
            sw.close();
        } catch (IOException e1) {
            e1.printStackTrace();
        }
    }
}
