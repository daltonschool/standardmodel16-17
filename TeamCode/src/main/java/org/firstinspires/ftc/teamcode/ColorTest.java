package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

@Autonomous(name="Beacon Color Test", group="Tests")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Utils.showToast("Beacon color test", 0);
        ColorSensor leftLineColor = hardwareMap.colorSensor.get("left line color");
        leftLineColor.setI2cAddress(I2cAddr.create8bit(0x6C));
        leftLineColor.enableLed(true);
        ColorSensor rightLineColor = hardwareMap.colorSensor.get("right line color");
        rightLineColor.setI2cAddress(I2cAddr.create8bit(0x4C));
        rightLineColor.enableLed(true);
        while (opModeIsActive()) {
            telemetry.addData("left-r", leftLineColor.red());
            telemetry.addData("left-g", leftLineColor.green());
            telemetry.addData("left-b", leftLineColor.blue());
            telemetry.addData("right-r", rightLineColor.red());
            telemetry.addData("right-g", rightLineColor.green());
            telemetry.addData("right-b", rightLineColor.blue());

            telemetry.update();
            idle();
        }
    }
}
