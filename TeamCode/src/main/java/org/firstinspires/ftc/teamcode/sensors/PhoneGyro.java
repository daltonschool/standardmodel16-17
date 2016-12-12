package org.firstinspires.ftc.teamcode.sensors;

import android.content.Context;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class PhoneGyro extends Sensor implements SensorEventListener {

    private android.hardware.Sensor _sensor;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;

    private float _rotationCurrent;

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
        // This timestep's delta rotation to be multiplied by the current rotation
        // after computing it from the gyro sample data.
        if (timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            final float EPSILON = 0.01f;

            // Axis of the rotation sample, not normalized yet.
            float axisX = event.values[0];
            float axisY = event.values[1];
            float axisZ = event.values[2];

            // Calculate the angular speed of the sample
            float omegaMagnitude = (float)Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            // Normalize the rotation vector if it's big enough to get the axis
            // (that is, EPSILON should represent your maximum allowable margin of error)
            if (omegaMagnitude > 0.01f) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            // Integrate around this axis with the angular speed by the timestep
            // in order to get a delta rotation from this sample over the timestep
            // We will convert this axis-angle representation of the delta rotation
            // into a quaternion before turning it into the rotation matrix.
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
            float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;
        }
        timestamp = event.timestamp;
        float[] deltaRotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
        // User code should concatenate the delta rotation we computed with the current rotation
        // in order to get the updated rotation.
        // rotationCurrent = rotationCurrent * deltaRotationMatrix;
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
