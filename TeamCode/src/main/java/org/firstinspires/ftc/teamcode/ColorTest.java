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

        MRColorSensor leftLineColor = new MRColorSensor(hardwareMap.i2cDeviceSynch.get("left line color"), I2cAddr.create8bit(0x6C));
        MRColorSensor rightLineColor = new MRColorSensor(hardwareMap.i2cDeviceSynch.get("right line color"), I2cAddr.create8bit(0x4C));
        ColorSensor beaconColor = hardwareMap.colorSensor.get("beacon color");
        beaconColor.enableLed(false);

        while (opModeIsActive()) {
            /*telemetry.addData("left-r", leftLineColor.red());
            telemetry.addData("left-g", leftLineColor.green());
            telemetry.addData("left-b", leftLineColor.blue());
            telemetry.addData("left-a", leftLineColor.alpha());
            telemetry.addData("left-q", Utils.intelIsBetterThanQualcomm((ModernRoboticsI2cColorSensor) leftLineColor, 0x4 + 0x11));

            //((ModernRoboticsI2cColorSensor)leftLineColor).
            telemetry.addData("right-r", rightLineColor.red());
            telemetry.addData("right-g", rightLineColor.green());
            telemetry.addData("right-b", rightLineColor.blue());
            telemetry.addData("right-a", rightLineColor.alpha());*/

            telemetry.addData("left-w", leftLineColor.whiteReading());
            telemetry.addData("right-w", rightLineColor.whiteReading());
            telemetry.addData("beaconColor-r", beaconColor.red());
            telemetry.addData("beaconColor-g", beaconColor.green());
            telemetry.addData("beaconColor-b", beaconColor.blue());

            telemetry.update();
            idle();
        }
    }
}
