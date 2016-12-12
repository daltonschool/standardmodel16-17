package org.firstinspires.ftc.teamcode.options;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.lang.reflect.Field;

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
    
    public static Field[] getOptionFields() {
        Options newOptions = new Options();
        Class c = Options.class;
        return c.getDeclaredFields();
    }

    public static Option getOptionAnnotationForField(Field f) {
        Option annotation = null;
        for (Annotation a : f.getAnnotations()) {
            if (a instanceof Option) {
                // we found it! set the variable and escape
                annotation = (Option)a;
                break;
            }
        }
        return annotation; // if we get here without having found it, we will just return null
    }

    public static Field getFieldFromName(String fieldName) {
        for (Field f : getOptionFields()) {
            if (f.getName().equals(fieldName)) {
                return f;
            }
        }
        return null;
    }

    public static String getPrettyName(String field) {
        Field f = getFieldFromName(field);
        if (f == null) {
            return "(null)";
        }
        return getOptionAnnotationForField(f).prettyName();
    }
}
