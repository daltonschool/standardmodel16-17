package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

/**
 * Created by student on 2/26/17.
 */
public class MoveForwardTimeTask extends Task {
    public MoveForwardTimeTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.leftMotors(1.0f);
        Robot.rightMotors(1.0f);

        Thread.sleep((int) extra);

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}