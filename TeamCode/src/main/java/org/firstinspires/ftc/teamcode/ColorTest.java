package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

@Autonomous(name="Color Test", group="Tests")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Utils.showToast("Color test", 0);

        /*ColorSensor leftLineColor = hardwareMap.colorSensor.get("left line color");
        leftLineColor.setI2cAddress(I2cAddr.create8bit(0x6C));
        leftLineColor.enableLed(true);
        ColorSensor rightLineColor = hardwareMap.colorSensor.get("right line color");
        rightLineColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        rightLineColor.enableLed(true);*/

        //MRColorSensor leftLineColor = new MRColorSensor(hardwareMap.i2cDeviceSynch.get("left line color"), I2cAddr.create8bit(0x6C));
        //MRColorSensor rightLineColor = new MRColorSensor(hardwareMap.i2cDeviceSynch.get("right line color"), I2cAddr.create8bit(0x4C));
        ColorSensor beaconColor = hardwareMap.colorSensor.get("left beacon color");
        beaconColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        beaconColor.enableLed(false);

        while (opModeIsActive()) {
            telemetry.addData("beaconColor-r", beaconColor.red());
            telemetry.addData("beaconColor-g", beaconColor.green());
            telemetry.addData("beaconColor-b", beaconColor.blue());

            telemetry.update();
            idle();
        }
    }
}
