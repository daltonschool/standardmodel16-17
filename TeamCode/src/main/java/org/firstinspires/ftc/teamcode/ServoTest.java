package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Servo Test", group="Tests")
public class ServoTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo;
        servo = hardwareMap.servo.get("beacon servo");

        waitForStart();

        servo.setPosition(0.0);
        Thread.sleep(3000);
        servo.setPosition(1.0);
        Thread.sleep(3000);
        servo.setPosition(0.5);
        Thread.sleep(3000);
        servo.setPosition(1.0);
        Thread.sleep(3000);
    }
}
