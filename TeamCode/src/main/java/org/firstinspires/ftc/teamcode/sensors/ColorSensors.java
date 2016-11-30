package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;
import android.os.StrictMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by student on 11/24/16.
 */
public class ColorSensors extends OpMode{
    public static ColorSensor cSensor = null;


    @Override
    public void init() {

    }

    public double getColorSensorVal (String cSensorName) {
        cSensor = hardwareMap.colorSensor.get(cSensorName);
        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV((cSensor.red() * 255) / 800, (cSensor.green() * 255) / 800, (cSensor.blue() * 255) / 800, hsvValues);
        return hsvValues[1];
    }

    public double getAlphaVal (String cSensorName) {
        cSensor = hardwareMap.colorSensor.get(cSensorName);
        return cSensor.alpha();
    }

    @Override
    public void loop() {

    }


}
