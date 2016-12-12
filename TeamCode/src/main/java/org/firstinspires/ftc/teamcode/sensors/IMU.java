package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * IMU = fancy name for accelerometer, gyroscope, and compass combo
 */
public class IMU extends Sensor {
    private Orientation angles = null;
    private Acceleration gravity = null;

    private BNO055IMU _imu;

    public IMU(BNO055IMU imu) {
        _imu = imu;
    }

    @Override
    public void init() {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        _imu.initialize(imuParameters);
    }

    @Override
    public boolean ping() {
        return true;
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
        return "BNO055 IMU";
    }

    @Override
    public String uniqueName() {
        return name();
    }

    public Orientation getZYXOrientation() {
        return angles;
    }

    public Acceleration getGravity() {
        return gravity;
    }

    public double getMagnitude() {
        Acceleration gravity = _imu.getGravity();
        return Math.sqrt(gravity.xAccel*gravity.xAccel
                + gravity.yAccel*gravity.yAccel
                + gravity.zAccel*gravity.zAccel);
    }

    public int getHeading() {
        return Math.round(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void update() {
        angles = _imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        gravity = _imu.getGravity();
    }
}
