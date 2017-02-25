package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class ActuallyGoOnLineTask extends Task {
    public ActuallyGoOnLineTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        OpticalDistanceSensor sensorToTest = Robot.centerLineLight;

        boolean goingBack = true;


        while (sensorToTest.getLightDetected() <= Robot.ODS_BLACK_VALUE) {
            while (true) {
                if (goingBack) {
                    Robot.leftMotors(-0.28f);
                    Robot.rightMotors(-0.28f);
                } else {
                    Robot.leftMotors(0.28f);
                    Robot.rightMotors(0.28f);
                }
                if (sensorToTest.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                    goingBack = !goingBack;
                }
            }
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
