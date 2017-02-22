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
        ColorSensor leftBeaconColor = hardwareMap.colorSensor.get("left beacon color");
        leftBeaconColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        leftBeaconColor.enableLed(false);

        ColorSensor rightBeaconColor = hardwareMap.colorSensor.get("right beacon color");
        rightBeaconColor.enableLed(false);

        while (opModeIsActive()) {
            telemetry.addData("leftBeaconColor-r", leftBeaconColor.red());
            telemetry.addData("leftBeaconColor-g", leftBeaconColor.green());
            telemetry.addData("leftBeaconColor-b", leftBeaconColor.blue());

            telemetry.addData("rightBeaconColor-r", rightBeaconColor.red());
            telemetry.addData("rightBeaconColor-g", rightBeaconColor.green());
            telemetry.addData("rightBeaconColor-b", rightBeaconColor.blue());

            telemetry.update();
            idle();
        }
    }
}
