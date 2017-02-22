package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class MoveBackUntilFrontLineTask extends Task {
    public MoveBackUntilFrontLineTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {

        Robot.leftMotors(-0.28f);
        Robot.rightMotors(-0.28f);

        while (true) {
            if (Robot.leftLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                break;
            }
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
