package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class WaitTask extends Task {
    long waitTime;

    public WaitTask(Object e) {
        super(e);
        waitTime = (int)e;
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Thread.sleep(waitTime);
    }
}
