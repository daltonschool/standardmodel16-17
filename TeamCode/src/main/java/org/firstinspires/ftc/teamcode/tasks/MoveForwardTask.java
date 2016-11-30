package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class MoveForwardTask extends Task {
    public MoveForwardTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.moveForward_encoder((int) extra, 0.5f);
    }
}
