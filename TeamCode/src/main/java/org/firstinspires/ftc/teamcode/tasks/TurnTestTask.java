package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class TurnTestTask extends Task {
    public TurnTestTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.5f);
        while (true) {
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }
    }
}
