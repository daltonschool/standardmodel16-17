package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.Utils;

public class MRColorSensor extends Sensor {
    public I2cDeviceSynch _device;
    public I2cAddr _address;

    public MRColorSensor(I2cDeviceSynch device, I2cAddr address) {
        _device = device;
        _address = address;

        if (address != null) {
            _device.setI2cAddress(address);
        }

        _device.engage();
    }

    @Override
    public boolean ping() {
        return true;
    }

    @Override
    public byte firmwareRevision() {
        return _device.read8(0x00);
    }

    @Override
    public byte manufacturer() {
        return _device.read8(0x01);
    }

    @Override
    public byte sensorIDCode() {
        return _device.read8(0x02);
    }

    @Override
    public String name() {
        return "Modern Robotics Color Sensor";
    }

    @Override
    public String uniqueName() {
        return name() + " @ " + Utils.intToHexString(_address.get8Bit());
    }

    @Override
    public void init() {
        // turn on the led
        _device.write(0x03, new byte[] { 0x00 });
    }

    public void blackLevelCalibration() {
        _device.write8(0x03, 0x42);
    }

    public void whiteBalanceCalibration() {
        _device.write8(0x03, 0x43);
    }

    public byte whiteReading() {
        // TODO: care about the msb too
        return _device.read8(0x14);
    }

    public byte redReading() {
        // TODO: care about the msb too
        return _device.read8(0x0E);
    }

    public byte blueReading() {
        // TODO: care about the msb too
        return _device.read8(0x12);
    }
}
