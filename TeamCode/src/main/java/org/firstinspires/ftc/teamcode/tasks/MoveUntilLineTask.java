package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
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
        Robot.leftMotors(0.19f);
        Robot.rightMotors(0.19f);

        while (Robot.leftLineColor.whiteReading() < 5) {
            Robot.telemetry.addData("leftLineBlue", Robot.leftLineColor.whiteReading());
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
