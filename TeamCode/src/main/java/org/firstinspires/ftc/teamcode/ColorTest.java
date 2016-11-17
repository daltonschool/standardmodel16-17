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
        ColorSensor c = hardwareMap.colorSensor.get("x color");
        c.setI2cAddress(I2cAddr.create8bit(0x4C));
        c.enableLed(false);
        while (opModeIsActive()) {
            telemetry.addData("r", c.red());
            telemetry.addData("g", c.green());
            telemetry.addData("b", c.blue());

            telemetry.update();
            idle();
        }
    }
}
