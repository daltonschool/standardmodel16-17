package org.firstinspires.ftc.teamcode.tasks;

import android.util.Log;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class MoveUntilLineTask extends Task {
    public MoveUntilLineTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        OpticalDistanceSensor sensorToTest = Robot.centerLineLight;

        Robot.leftMotors(0.18f);
        Robot.rightMotors(0.18f);

        while (true) {
            if (sensorToTest.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                break;
            }
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
