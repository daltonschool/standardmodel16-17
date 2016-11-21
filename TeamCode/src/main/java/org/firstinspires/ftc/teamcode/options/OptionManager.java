package org.firstinspires.ftc.teamcode.options;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class OptionManager {
    public static Options currentOptions;
    public static String optionPath = "/sdcard/sm_options.json";

    public static void init() {
        currentOptions = load();
    }

    public static Options load() {
        Gson g = new Gson();
        File optionsFile = new File(optionPath);
        if (!optionsFile.exists()) {
            // options file doesn't exist...
            Options defaults = new Options();
            defaults.shotsOnly = false;
            defaults.startDelay = false;
            return defaults;
        }
        try {
            FileReader optionsFR = new FileReader(optionsFile);
            JsonReader reader = g.newJsonReader(optionsFR);
            Options o = g.fromJson(reader, Options.class);
            return o;
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return null;
    }

    public static void save(Options o) {
        Gson g = new Gson();
        String json = g.toJson(o);
        File optionsFile = new File(optionPath);
        FileWriter optionsFW = null;
        try {
            optionsFW = new FileWriter(optionsFile, false);
            optionsFW.write(json);
            optionsFW.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static String getPrettyName(String field) {
        switch (field) {
            case "shotsOnly":
                return "Shots only mode";
            case "startDelay":
                return "10 sec start delay";
            default:
                return field;
        }
    }
}
