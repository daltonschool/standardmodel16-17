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
        double voltage = Robot.voltageSensor.getVoltage();
        double power = 0.5f;
        if (voltage > 12.5) {
            power = 0.4f;
        }
        Robot.moveForward_encoder((int) extra, power);
    }
}
