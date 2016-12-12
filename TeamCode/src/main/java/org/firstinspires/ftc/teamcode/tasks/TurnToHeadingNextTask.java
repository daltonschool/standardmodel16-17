package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class TurnToHeadingNextTask extends Task {
    public TurnToHeadingNextTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        float currentHeading = Robot.imu.getHeading();
        int targetHeading = (int) extra;

        while (true) {
            Robot.telemetry.addData("Heading", currentHeading);
            Robot.telemetry.update();

            float distanceToTarget = Math.abs(currentHeading - targetHeading);
        }
    }
}
