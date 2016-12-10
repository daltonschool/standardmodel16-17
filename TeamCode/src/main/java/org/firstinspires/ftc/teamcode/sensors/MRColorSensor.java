package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class MRColorSensor {
    public I2cDeviceSynch _device;
    public I2cAddr address;

    public MRColorSensor(I2cDeviceSynch device, I2cAddr addr) {
        _device = device;

        if (addr != null) {
            _device.setI2cAddress(addr);
        }

        _device.engage();

        // turn on the led
        _device.write(0x03, new byte[] { 0x00 });
    }

    public byte whiteReading() {
        // TODO: care about the msb too
        return _device.read8(0x14);
    }
}
