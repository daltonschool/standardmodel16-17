package org.firstinspires.ftc.teamcode.sensors;

import android.content.Context;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class PhoneGyro extends Sensor implements SensorEventListener {

    private android.hardware.Sensor _sensor;

    private float _currentHeading = 0;

    public PhoneGyro() {
        SensorManager sensorManager = (SensorManager) Robot.appContext.getSystemService(Context.SENSOR_SERVICE);
        _sensor = sensorManager.getDefaultSensor(android.hardware.Sensor.TYPE_GYROSCOPE);
        sensorManager.registerListener(this, _sensor, SensorManager.SENSOR_DELAY_NORMAL);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean ping() {
        return true; // if this fails to initialize then we probably have bigger problems than a sensor...
    }

    @Override
    public byte firmwareRevision() {
        return 0;
    }

    @Override
    public byte manufacturer() {
        return 0;
    }

    @Override
    public byte sensorIDCode() {
        return 0;
    }

    @Override
    public String name() {
        return "Android Sensor - Gyroscope";
    }

    @Override
    public String uniqueName() {
        return name();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        _currentHeading = event.values[0];
    }

    @Override
    public void onAccuracyChanged(android.hardware.Sensor sensor, int accuracy) {
        // what is this
        // i don't know
        // hopefully leaving it empty doesn't break things
    }

    public int getHeading() {
        return -1 * Math.round(_currentHeading); // multiply by -1 to match existing gyro orientation system
    }
}
