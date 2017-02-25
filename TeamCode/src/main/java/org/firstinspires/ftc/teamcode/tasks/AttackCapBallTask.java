package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class AttackCapBallTask extends Task {
    public AttackCapBallTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.leftMotors(1.0f);
        Robot.rightMotors(-1.0f);

        Thread.sleep(2000);

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
