package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.IMU;

@Autonomous(name="Gyro Test", group="Tests")
public class GyroTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.init();

        telemetry.addData("Status", "spook");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Heading", imu.getHeading());
            telemetry.update();

            imu.update();
        }

        requestOpModeStop();
    }
}
